// session_client.cpp - reference host client for the SensorSync-Logger session API.
//
// Usage from an acquisition driver:
//     SensorSyncSession s;
//     s.open("/dev/serial/by-id/usb-Teensyduino_...");   // by-id survives re-enumeration
//     s.start("/data/logs/run_0007.log",                 // one file per run, counts from 0
//             {{0, 25.0}, {1, 2.0}});                    // per-channel trigger Hz, decided in the field
//     ... acquisition ...
//     s.stop();                                          // board -> idle, counts cleared
//
// The freq list is optional; {ch, 0} switches a channel off, omitted channels keep
// their config.h rate. It rides inside the START command ("START freq=0:25,1:2 <path>"),
// so the board receives the rate request and session identity together. The board
// echoes the EFFECTIVE rates in the "#TRIG" header; invalid
// entries answer "#ERR,bad_freq,..." while keeping the old rate.
//
// start() flushes stale input, sends "START <path>", opens <path>, and spawns a
// reader thread that copies the serial stream into the file until stop().
// The board cannot write the host filesystem: <path> is echoed into the log's
// "#SESSION,START,<path>" line; this client does the file I/O.
//
// A port drop, unexpected #IDLE, repeated START, #ERR or reported event loss
// latches an integrity failure. The caller must end this camera/time epoch;
// an automatic board restart would destroy the original frame association.
//
// Protocol (newline-terminated, 115200 8N1 - do not change the baud):
//   host -> board:  "START [freq=<ch>:<hz>,...] [path]" | "STOP" | "s" (status) | "h" (header)
//   board -> host:  "#SESSION,START,<path>", header, P/T/S/Z data + "#H" health,
//                   "#HFINAL ...", "#SESSION,STOP,<path>"; "#IDLE,up=<ms>" while idle
//
// Build demo:  g++ -std=c++20 -O2 -pthread -DSESSION_CLIENT_DEMO session_client.cpp -o session_client
// Run demo:    ./session_client /dev/ttyACM0 /tmp/run.log 5 25    (trig[0] at 25 Hz)

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include "session_protocol.h"

class SensorSyncSession {
public:
	~SensorSyncSession() {
		stop();
		if (out_) ::fclose(out_); // also covers an allocation exception during start()
		std::lock_guard<std::mutex> lk(io_mtx_);
		closePortLocked();
	}

	// open + configure the port as raw 115200 8N1; false on failure.
	// prefer a /dev/serial/by-id/ path: it survives USB re-enumeration.
	bool open(const char *port) {
		std::lock_guard<std::mutex> lk(io_mtx_);
		port_path_ = port;
		closePortLocked();
		return openPortLocked();
	}

	// begin a run: fresh counts from 0, stream captured to `path`.
	// freqs: per-channel trigger rates {ch, hz} decided in the field (exposure
	// time known -> rate computed); {ch, 0} = channel off; empty = keep defaults
	bool start(const std::string &path, const std::vector<std::pair<int, double>> &freqs = {}) {
		if (running_) {
			return false;
		}
		if (path.find_first_of("\r\n") != std::string::npos || path.find('\0') != std::string::npos) return false;
		const int log_fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
		if (log_fd < 0) return false;
		out_ = ::fdopen(log_fd, "wb");
		if (!out_) {
			::close(log_fd);
			return false;
		}
		session_path_ = path;
		freq_spec_.clear();
		for (const auto &f : freqs) {
			char item[48];
			snprintf(item, sizeof(item), "%s%d:%g", freq_spec_.empty() ? "" : ",", f.first, f.second);
			freq_spec_ += item;
		}
		{
			std::lock_guard<std::mutex> lk(io_mtx_);
			if (fd_ < 0) {
				::fclose(out_);
				out_ = nullptr;
				return false;
			}
			// stale #IDLE lines buffered since open() must not precede #SESSION,START
			if (tcflush(fd_, TCIFLUSH) != 0) {
				::fclose(out_);
				out_ = nullptr;
				return false;
			}
			if (!sendStartLocked()) {
				writeCommandLocked("STOP\n"); // best effort after an ambiguous/partial START write
				::fclose(out_);
				out_ = nullptr;
				return false;
			}
		}
		write_error_ = false;
		protocol_error_ = false;
		start_ack_ = false;
		stop_ack_ = false;
		protocol_ = {};
		line_len_ = 0;
		stopping_ = false;
		running_ = true;
		try {
			reader_ = std::thread(&SensorSyncSession::readerLoop, this);
		} catch (...) {
			stop(); // even a thread-creation failure must stop pulses already requested
			return false;
		}
		const auto deadline = Clock::now() + std::chrono::seconds(3);
		while (!start_ack_ && ok() && Clock::now() < deadline) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		if (!start_ack_ || !ok()) {
			protocol_error_ = true;
			stop();
			return false;
		}
		return true;
	}

	// end the run: board -> idle; drain the tail, close the file
	void stop() {
		if (!running_) {
			return;
		}
		stopping_ = true;  // post-STOP "#IDLE" is expected
		{
			std::lock_guard<std::mutex> lk(io_mtx_);
			if (fd_ >= 0) {
				const char *cmd = "STOP\n";
				if (!writeCommandLocked(cmd)) protocol_error_ = true;
			} else {
				protocol_error_ = true;
			}
		}
		// let "#SESSION,STOP" land in the file before tearing the reader down
		const auto deadline = Clock::now() + std::chrono::seconds(3); // firmware drains + emits final health before STOP
		while (!stop_ack_ && !write_error_ && Clock::now() < deadline) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		if (!stop_ack_) protocol_error_ = true;
		running_ = false;
		if (reader_.joinable()) {
			reader_.join();
		}
		if (out_) {
			if (::fflush(out_) != 0 || ::fdatasync(::fileno(out_)) != 0) write_error_ = true;
			if (::fclose(out_) != 0) write_error_ = true;
			out_ = nullptr;
		}
	}

	bool ok() const {
		return !write_error_ && !protocol_error_;
	}

	// errno of the last failed open() port setup (0 = no failure); lets the
	// caller tell EACCES (no tty permission) from EBUSY (port held) from
	// ENOENT (board gone) instead of guessing
	int lastErrno() const {
		return last_errno_;
	}

private:
	using Clock = std::chrono::steady_clock;

	// --- port lifecycle (io_mtx_ held) ---------------------------------------
	void closePortLocked() {
		if (fd_ >= 0) {
			::close(fd_);
			fd_ = -1;
		}
	}

	bool openPortLocked() {
		fd_ = ::open(port_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
		if (fd_ < 0) {
			last_errno_ = errno;
			return false;
		}
		termios tio{};
		if (tcgetattr(fd_, &tio) != 0) {
			last_errno_ = errno;  // capture before close() can clobber errno
			closePortLocked();
			return false;
		}
		cfmakeraw(&tio);
		tio.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
		tio.c_cflag &= ~CRTSCTS;
#endif
		cfsetispeed(&tio, B115200);
		cfsetospeed(&tio, B115200);
		tio.c_cflag |= (CLOCAL | CREAD);
		tio.c_cc[VMIN] = 0;
		tio.c_cc[VTIME] = 1;	 // 0.1 s read timeout -> reader can poll running_
		// Drop O_NONBLOCK so VMIN/VTIME govern reads; failure would break EOF detection.
		if (fcntl(fd_, F_SETFL, 0) != 0 || tcsetattr(fd_, TCSANOW, &tio) != 0) {
			last_errno_ = errno;
			closePortLocked();
			return false;
		}
		last_errno_ = 0;
		return true;
	}

	bool sendStartLocked() {
		// Board command storage is bounded; never allow silent truncation of its identity.
		std::string cmd = "START ";
		if (!freq_spec_.empty()) {
			cmd += "freq=" + freq_spec_ + " ";
		}
		cmd += session_path_ + "\n";
		if (cmd.size() > 255) return false;
		return writeCommandLocked(cmd);
	}

	bool writeCommandLocked(const std::string &cmd) {
		if (fd_ < 0) return false;
		std::size_t sent = 0;
		while (sent < cmd.size()) {
			const ssize_t n = ::write(fd_, cmd.data() + sent, cmd.size() - sent);
			if (n < 0 && errno == EINTR) continue;
			if (n <= 0) return false;
			sent += static_cast<std::size_t>(n);
		}
		return true;
	}

	// --- reader thread -------------------------------------------------------
	void readerLoop() {
		char buf[512];
		int fast_zero = 0;	// EOF detector: real VTIME timeouts take ~100 ms
		while (running_) {
			int fd;
			{
				std::lock_guard<std::mutex> lk(io_mtx_);
				fd = fd_;
			}
			if (fd < 0) {
				protocol_error_ = true;
				break; // a new clock epoch must not be silently joined to this recording
			}
			const auto t0 = Clock::now();
			const ssize_t n = ::read(fd, buf, sizeof(buf));
			if (n > 0) {
				fast_zero = 0;
				if (::fwrite(buf, 1, static_cast<std::size_t>(n), out_) != static_cast<std::size_t>(n) ||
				    ::fflush(out_) != 0) {
					write_error_ = true;
					break;
				}
				scanStream(buf, static_cast<size_t>(n));
			} else if (n < 0) {
				if (errno == EINTR) {
					continue;
				}
				// Unplug/reboot ends this epoch; never reopen into the same timing log.
				std::lock_guard<std::mutex> lk(io_mtx_);
				closePortLocked();
				protocol_error_ = true;
			} else {  // n == 0: VTIME timeout - or EOF, which returns instantly
				if (Clock::now() - t0 < std::chrono::milliseconds(20)) {
					if (++fast_zero >= 20) {  // sustained instant EOF = dead port
						fast_zero = 0;
						std::lock_guard<std::mutex> lk(io_mtx_);
						closePortLocked();
						protocol_error_ = true;
					}
				} else {
					fast_zero = 0;
				}
			}
		}
	}

	// Check acknowledgements, final counters and epoch continuity in complete lines.
	void scanStream(const char *buf, size_t n) {
		for (size_t i = 0; i < n; i++) {
			const char c = buf[i];
			if (c == '\n' || c == '\r') {
				const std::string_view line(line_, line_len_);
				for (const std::string_view prefix : {"#SESSION,START,", "#SESSION,STOP,"}) {
					if (line.starts_with(prefix) && line.substr(prefix.size()) != session_path_) protocol_error_ = true;
				}
				protocol_.observe(line, stopping_.load());
				if (protocol_.failed) protocol_error_ = true;
				if (protocol_.starts > 0) start_ack_ = true;
				if (protocol_.stop_received) stop_ack_ = true;
				line_len_ = 0;
			} else if (line_len_ < sizeof(line_)) {
				line_[line_len_++] = c;
			} else {
				protocol_error_ = true; // preserve bytes in the log, but never accept a truncated control record
			}
		}
	}

	std::string port_path_;
	std::string session_path_;
	std::string freq_spec_;	 // "0:25,1:2" - per-channel Hz for this run
	int fd_ = -1;
	int last_errno_ = 0;  // errno of the last failed port setup
	std::FILE *out_ = nullptr;
	std::thread reader_;
	std::mutex io_mtx_;	 // guards fd_ open/close/write
	std::atomic<bool> running_{ false };
	std::atomic<bool> stopping_{ false };
	std::atomic<bool> write_error_{ false };
	std::atomic<bool> protocol_error_{ false };
	std::atomic<bool> start_ack_{ false };
	std::atomic<bool> stop_ack_{ false };
	SensorSyncProtocol protocol_; // reader-thread-only; state mirrored through atomics above
	char line_[2048];
	size_t line_len_ = 0;
};


#ifdef SESSION_CLIENT_DEMO
// demo: ./session_client <port> <logfile> [seconds] [trig0_hz]
static std::atomic<bool> g_interrupted{ false };

int main(int argc, char **argv) {
	if (argc < 3) {
		fprintf(stderr, "usage: %s <port> <logfile> [seconds] [trig0_hz]\n", argv[0]);
		return 2;
	}
	const int secs = (argc > 3) ? atoi(argv[3]) : 5;
	std::vector<std::pair<int, double>> freqs;
	if (argc > 4) {
		freqs.emplace_back(0, atof(argv[4]));  // field-decided camera rate
	}
	SensorSyncSession s;
	if (!s.open(argv[1])) {
		fprintf(stderr, "open %s failed: %s\n", argv[1], strerror(s.lastErrno()));
		return 1;
	}
	if (!s.start(argv[2], freqs)) {
		fprintf(stderr, "start failed\n");
		return 1;
	}
	signal(SIGINT, [](int) { g_interrupted = true; });	// Ctrl-C still sends STOP
	fprintf(stderr, "logging to %s for %d s (Ctrl-C to stop early) ...\n", argv[2], secs);
	for (int i = 0; i < secs * 10 && !g_interrupted; i++) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	s.stop();
	if (!s.ok()) {
		fprintf(stderr, "warning: log file write error (disk full?)\n");
		return 1;
	}
	fprintf(stderr, "done.\n");
	return 0;
}
#endif
