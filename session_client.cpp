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
// so a watchdog re-START after a board reboot restores the same rates. The board
// echoes the EFFECTIVE rates in the "#TRIG" header - verify there, and invalid
// entries answer "#ERR,bad_freq,..." while keeping the old rate.
//
// start() flushes stale input, sends "START <path>", opens <path>, and spawns a
// reader thread that copies the serial stream into the file until stop().
// The board cannot write the host filesystem: <path> is echoed into the log's
// "#SESSION,START,<path>" line; this client does the file I/O.
//
// Recovery while a run is active:
//   - port drops (USB glitch / board reboot): reader re-opens the same path until
//     it returns, appending to the SAME file - prefer a /dev/serial/by-id/ path
//   - "#IDLE" seen in the stream (board rebooted to idle, or a lost START): the
//     watchdog re-sends START; the new "#SESSION,START" marker lands in the same
//     file and postprocess.py splits the sessions
//
// Protocol (newline-terminated, 115200 8N1 - do not change the baud):
//   host -> board:  "START [freq=<ch>:<hz>,...] [path]" | "STOP" | "s" (status) | "h" (header)
//   board -> host:  "#SESSION,START,<path>", header, P/T/S/Z data + "#H" health,
//                   "#SESSION,STOP,<path>"; "#IDLE,up=<ms>" while idle
//
// Build demo:  g++ -std=c++17 -O2 -pthread -DSESSION_CLIENT_DEMO session_client.cpp -o session_client
// Run demo:    ./session_client /dev/ttyACM0 /tmp/run.log 5 25    (trig[0] at 25 Hz)

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class SensorSyncSession {
public:
	~SensorSyncSession() {
		stop();
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
		out_.open(path, std::ios::binary | std::ios::trunc);
		if (!out_) {
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
				out_.close();
				return false;
			}
			// stale #IDLE lines buffered since open() must not precede #SESSION,START
			tcflush(fd_, TCIFLUSH);
			if (!sendStartLocked()) {
				out_.close();
				return false;
			}
		}
		write_error_ = false;
		line_len_ = 0;
		stopping_ = false;
		running_ = true;
		reader_ = std::thread(&SensorSyncSession::readerLoop, this);
		return true;
	}

	// end the run: board -> idle; drain the tail, close the file
	void stop() {
		if (!running_) {
			return;
		}
		stopping_ = true;  // mute the watchdog: post-STOP "#IDLE" is expected, not a reboot
		{
			std::lock_guard<std::mutex> lk(io_mtx_);
			if (fd_ >= 0) {
				const char *cmd = "STOP\n";
				const ssize_t wr = ::write(fd_, cmd, std::strlen(cmd));
				(void) wr;
			}
		}
		// let "#SESSION,STOP" land in the file before tearing the reader down
		std::this_thread::sleep_for(std::chrono::milliseconds(300));
		running_ = false;
		if (reader_.joinable()) {
			reader_.join();
		}
		out_.close();
	}

	bool ok() const {
		return !write_error_;  // false = log file write failed (e.g. disk full)
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
		fd_ = ::open(port_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
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
		cfsetispeed(&tio, B115200);
		cfsetospeed(&tio, B115200);
		tio.c_cflag |= (CLOCAL | CREAD);
		tio.c_cc[VMIN] = 0;
		tio.c_cc[VTIME] = 1;	 // 0.1 s read timeout -> reader can poll running_
		fcntl(fd_, F_SETFL, 0);	 // drop O_NONBLOCK; VMIN/VTIME govern reads now
		if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
			last_errno_ = errno;
			closePortLocked();
			return false;
		}
		last_errno_ = 0;
		return true;
	}

	bool sendStartLocked() {
		// freq rides in START so a watchdog re-START restores the same rates
		std::string cmd = "START ";
		if (!freq_spec_.empty()) {
			cmd += "freq=" + freq_spec_ + " ";
		}
		cmd += session_path_ + "\n";
		last_start_ = Clock::now();
		return fd_ >= 0 && ::write(fd_, cmd.data(), cmd.size()) == static_cast<ssize_t>(cmd.size());
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
			if (fd < 0) {  // port lost: retry the same path until it re-enumerates
				std::this_thread::sleep_for(std::chrono::milliseconds(500));
				std::lock_guard<std::mutex> lk(io_mtx_);
				if (running_ && openPortLocked()) {
					fast_zero = 0;
					// no tcflush here: a still-running session may already be streaming
				}
				continue;
			}
			const auto t0 = Clock::now();
			const ssize_t n = ::read(fd, buf, sizeof(buf));
			if (n > 0) {
				fast_zero = 0;
				out_.write(buf, n);
				out_.flush();
				if (!out_) {  // disk full / I/O error: surface, stop consuming
					write_error_ = true;
					break;
				}
				scanStream(buf, static_cast<size_t>(n));
			} else if (n < 0) {
				if (errno == EINTR) {
					continue;
				}
				// unplug / reboot: EIO or ENXIO -> drop the fd, reconnect above
				std::lock_guard<std::mutex> lk(io_mtx_);
				closePortLocked();
			} else {  // n == 0: VTIME timeout - or EOF, which returns instantly
				if (Clock::now() - t0 < std::chrono::milliseconds(20)) {
					if (++fast_zero >= 20) {  // sustained instant EOF = dead port
						fast_zero = 0;
						std::lock_guard<std::mutex> lk(io_mtx_);
						closePortLocked();
					}
				} else {
					fast_zero = 0;
				}
			}
		}
	}

	// watch the stream for "#IDLE": board is not in a session (reboot, or a lost
	// START) -> re-send START, rate-limited. Data lines never start with #IDLE.
	void scanStream(const char *buf, size_t n) {
		for (size_t i = 0; i < n; i++) {
			const char c = buf[i];
			if (c == '\n' || c == '\r') {
				if (line_len_ >= 5 && memcmp(line_, "#IDLE", 5) == 0) {
					maybeResendStart();
				}
				line_len_ = 0;
			} else if (line_len_ < sizeof(line_)) {
				line_[line_len_++] = c;	 // overlong lines just stop matching #IDLE
			}
		}
	}

	void maybeResendStart() {
		if (stopping_) {
			return;
		}
		std::lock_guard<std::mutex> lk(io_mtx_);
		if (Clock::now() - last_start_ > std::chrono::seconds(2)) {
			sendStartLocked();
		}
	}

	std::string port_path_;
	std::string session_path_;
	std::string freq_spec_;	 // "0:25,1:2" - per-channel Hz for this run
	int fd_ = -1;
	int last_errno_ = 0;  // errno of the last failed port setup
	std::ofstream out_;
	std::thread reader_;
	std::mutex io_mtx_;	 // guards fd_ open/close/write
	std::atomic<bool> running_{ false };
	std::atomic<bool> stopping_{ false };
	std::atomic<bool> write_error_{ false };
	char line_[16];
	size_t line_len_ = 0;
	Clock::time_point last_start_{};
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
