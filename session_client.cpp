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
// open() configures raw/no-echo, terminates any residual command, stops the board,
// and verifies a fresh round trip followed by idle. Call it BEFORE camera arming.
// startupLog() retains the bounded pre-session transcript separately from data.
// start() opens <path>, flushes stale idle input, sends "START <path>", and spawns
// a reader thread that copies the serial stream into the file until stop().
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
#include <cmath>
#include <climits>
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
#include <poll.h>
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

	// Open, configure raw 115200 8N1, and synchronize to idle before camera arming.
	// This sends STOP; the caller must own the board, with no other serial clients.
	// prefer a /dev/serial/by-id/ path: it survives USB re-enumeration.
	bool open(const char *port) {
		std::lock_guard<std::mutex> lk(io_mtx_);
		if (running_) return fail("cannot reopen a running session", EBUSY);
		startup_log_.clear();
		last_error_.clear();
		port_path_ = port;
		closePortLocked();
		return openPortLocked();
	}

	// begin a run: fresh counts from 0, stream captured to `path`.
	// freqs: per-channel trigger rates {ch, hz} decided in the field (exposure
	// time known -> rate computed); {ch, 0} = channel off; empty = keep defaults
	bool start(const std::string &path, const std::vector<std::pair<int, double>> &freqs = {}) {
		last_error_.clear();
		if (running_) {
			return fail("session already running", EBUSY);
		}
		if (path.size() > 223 || path.find_first_of("\r\n") != std::string::npos || path.find('\0') != std::string::npos)
			return fail("invalid session path", EINVAL);
		for (const auto &f : freqs) {
			if (f.first < 0 || f.first >= 4 || !std::isfinite(f.second) || f.second < 0.0)
				return fail("invalid trigger channel or frequency", EINVAL);
		}
		const int log_fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
		if (log_fd < 0) return fail("create timing log", errno);
		out_ = ::fdopen(log_fd, "wb");
		if (!out_) {
			const int err = errno;
			::close(log_fd);
			return fail("open timing log stream", err);
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
				return fail("serial port is not open", EBADF);
			}
			// stale #IDLE lines buffered since open() must not precede #SESSION,START
			if (tcflush(fd_, TCIFLUSH) != 0) {
				const int err = errno;
				::fclose(out_);
				out_ = nullptr;
				return fail("flush idle input before START", err);
			}
			if (!sendStartLocked()) {
				const int err = errno;
				writeCommandLocked("STOP\n"); // best effort after an ambiguous/partial START write
				::fclose(out_);
				out_ = nullptr;
				return fail("send START command", err);
			}
		}
		write_error_ = false;
		protocol_error_ = false;
		start_ack_ = false;
		stop_ack_ = false;
		protocol_ = {};
		protocol_detail_.clear();
		line_len_ = 0;
		stopping_ = false;
		running_ = true;
		try {
			reader_ = std::thread(&SensorSyncSession::readerLoop, this);
		} catch (...) {
			stop(); // even a thread-creation failure must stop pulses already requested
			return fail("create serial reader thread");
		}
		const auto deadline = Clock::now() + std::chrono::seconds(3);
		while (!start_ack_ && ok() && Clock::now() < deadline) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		if (!start_ack_ || !ok()) {
			const bool missing_ack = !start_ack_;
			protocol_error_ = true;
			stop();
			// The reader is joined: its first protocol failure is safe to inspect.
			if (!protocol_detail_.empty()) return fail("START rejected: " + protocol_detail_, EPROTO);
			return fail(missing_ack ? "START acknowledgement missing (see timing log)"
			                        : "START serial/log integrity failure (see timing log)");
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

	// Diagnostics for failed open()/start(); inspect after the call returns.
	int lastErrno() const {
		return last_errno_;
	}
	const std::string &lastError() const { return last_error_; }
	// Raw synchronization replies, bounded to 64 KiB. Retain in the application's
	// diagnostic log, not in the subsequent timing data file.
	const std::string &startupLog() const { return startup_log_; }

private:
	using Clock = std::chrono::steady_clock;

	bool fail(std::string_view reason, int err = 0) {
		last_errno_ = err;
		last_error_ = reason;
		if (err) last_error_ += ": " + std::string(std::strerror(err));
		return false;
	}

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
			return fail("open serial port", errno);
		}
		termios tio{};
		if (tcgetattr(fd_, &tio) != 0) {
			const int err = errno;
			closePortLocked();
			return fail("read serial settings", err);
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
		// Disable echo BEFORE clearing queued echo bytes or sending anything.
		// Keep O_NONBLOCK during startup so reads and writes obey its deadline.
		if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
			const int err = errno;
			closePortLocked();
			return fail("apply raw serial settings", err);
		}
		if (!synchronizeLocked()) {
			closePortLocked();
			return false;
		}
		// Normal recording uses VMIN/VTIME for the existing reader's EOF checks.
		if (fcntl(fd_, F_SETFL, 0) != 0) {
			const int err = errno;
			closePortLocked();
			return fail("enable timed serial reads", err);
		}
		last_errno_ = 0;
		return true;
	}

	bool waitStartupIoLocked(short events, Clock::time_point deadline) {
		while (Clock::now() < deadline) {
			pollfd pfd{fd_, events, 0};
			const int ready = ::poll(&pfd, 1, 100);
			if (ready < 0) {
				if (errno == EINTR) continue;
				return fail("startup serial poll", errno);
			}
			if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
				return fail("serial port lost during startup synchronization", EIO);
			if (pfd.revents & events) return true;
		}
		return fail("startup synchronization timed out waiting for barrier/idle", ETIMEDOUT);
	}

	bool synchronizeLocked() {
		// This clears host queues, not the firmware's persistent cmd[]/cmdlen.
		if (tcflush(fd_, TCIOFLUSH) != 0) return fail("flush startup serial queues", errno);
		static std::atomic<unsigned long long> sequence{0};
		const std::string marker = "__SENSORSYNC_SYNC_" + std::to_string(::getpid()) + "_" +
			std::to_string(Clock::now().time_since_epoch().count()) + "_" + std::to_string(++sequence);
		const std::string barrier_reply = "#ERR,unknown_command," + marker;
		const std::string completion_reply = barrier_reply + "_DONE";
		SensorSyncStartupProtocol startup{barrier_reply, completion_reply};
		// Terminate the residual line, then stop any old session. The fresh marker
		// proves earlier commands/replies were processed; status must report idle.
		// A second marker confirms the status command was consumed, even if an
		// idle heartbeat preceded its reply. Never request START in this phase.
		const std::string commands = "\nSTOP\n" + marker + "\ns\n" + marker + "_DONE\n";
		const auto deadline = Clock::now() + std::chrono::seconds(3);
		std::size_t sent = 0;
		while (sent < commands.size()) {
			if (!waitStartupIoLocked(POLLOUT, deadline)) return false;
			const ssize_t n = ::write(fd_, commands.data() + sent, commands.size() - sent);
			if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
			if (n <= 0) return fail("write startup synchronization commands", n < 0 ? errno : EIO);
			sent += static_cast<std::size_t>(n);
		}

		char buf[512], line[2048];
		std::size_t line_len = 0;
		for (;;) {
			if (!waitStartupIoLocked(POLLIN, deadline)) return false;
			const ssize_t n = ::read(fd_, buf, sizeof(buf));
			if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
			if (n <= 0) return fail("read startup synchronization replies", n < 0 ? errno : EIO);
			const auto size = static_cast<std::size_t>(n);
			if (startup_log_.size() + size > 64 * 1024)
				return fail("startup synchronization exceeded 64 KiB diagnostic limit", EOVERFLOW);
			startup_log_.append(buf, size);
			for (std::size_t i = 0; i < size; ++i) {
				if (buf[i] == '\n') {
					// Wait for LF even when CRLF is split across reads: a leftover
					// LF must not become the first byte of the capture log.
					std::string_view reply(line, line_len);
					if (reply.ends_with('\r')) reply.remove_suffix(1);
					startup.observe(reply);
					if (startup.failed) return fail("unexpected reply after startup barrier: " + std::string(reply), EPROTO);
					line_len = 0;
				} else if (line_len < sizeof(line)) {
					line[line_len++] = buf[i];
				} else {
					return fail("startup reply exceeds 2048-byte line limit", EOVERFLOW);
				}
			}
			// Process the whole read, including any fragmented trailing reply.
			if (startup.completed && line_len == 0) return true;
		}
	}

	bool sendStartLocked() {
		// Board command storage is bounded; never allow silent truncation of its identity.
		std::string cmd = "START ";
		if (!freq_spec_.empty()) {
			cmd += "freq=" + freq_spec_ + " ";
		}
		cmd += session_path_ + "\n";
		if (cmd.size() > 255) { errno = EMSGSIZE; return false; }
		return writeCommandLocked(cmd);
	}

	bool writeCommandLocked(const std::string &cmd) {
		if (fd_ < 0) { errno = EBADF; return false; }
		std::size_t sent = 0;
		while (sent < cmd.size()) {
			const ssize_t n = ::write(fd_, cmd.data() + sent, cmd.size() - sent);
			if (n < 0 && errno == EINTR) continue;
			if (n <= 0) { if (n == 0) errno = EIO; return false; }
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
				bool bad_line = false;
				for (const std::string_view prefix : {"#SESSION,START,", "#SESSION,STOP,"}) {
					if (line.starts_with(prefix) && line.substr(prefix.size()) != session_path_) bad_line = true;
				}
				const bool already_failed = protocol_.failed;
				protocol_.observe(line, stopping_.load());
				bad_line = bad_line || (!already_failed && protocol_.failed);
				if (bad_line) {
					protocol_error_ = true;
					if (protocol_detail_.empty()) protocol_detail_ = std::string(line);
				}
				if (protocol_.starts > 0) start_ack_ = true;
				if (protocol_.stop_received) stop_ack_ = true;
				line_len_ = 0;
			} else if (line_len_ < sizeof(line_)) {
				line_[line_len_++] = c;
			} else {
				protocol_error_ = true; // preserve bytes in the log, but never accept a truncated control record
				if (protocol_detail_.empty()) protocol_detail_ = "control record exceeds 2048 bytes";
			}
		}
	}

	std::string port_path_;
	std::string session_path_;
	std::string freq_spec_;	 // "0:25,1:2" - per-channel Hz for this run
	int fd_ = -1;
	int last_errno_ = 0;
	std::string last_error_;
	std::string startup_log_;
	std::string protocol_detail_; // reader-thread-only until stop() joins it
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
// seconds=0 records until SIGINT/SIGTERM. Optional status file is for record_ctl.bash.
static volatile std::sig_atomic_t g_interrupted = 0;

static bool writeStatus(const char *path, const char *value) {
	if (!path) return true;
	const std::string tmp = std::string(path) + ".tmp";
	FILE *f = ::fopen(tmp.c_str(), "w");
	if (!f) return false;
	const bool written = ::fprintf(f, "%s\n", value) > 0;
	const bool closed = ::fclose(f) == 0;
	return written && closed && ::rename(tmp.c_str(), path) == 0;
}

static bool parseFreqs(const char *arg, std::vector<std::pair<int, double>> &freqs) {
	std::string spec = arg;
	if (!spec.starts_with("freq=")) spec = "0:" + spec; // retain the original trig0_hz argument
	else spec.erase(0, 5);
	if (spec.empty()) return true;
	std::size_t pos = 0;
	while (pos < spec.size()) {
		const auto end = spec.find(',', pos);
		const std::string item = spec.substr(pos, end == std::string::npos ? end : end - pos);
		char *colon = nullptr, *last = nullptr;
		const long ch = strtol(item.c_str(), &colon, 10);
		if (colon == item.c_str() || *colon != ':' || ch < 0 || ch >= 4) return false;
		const double hz = strtod(colon + 1, &last);
		if (last == colon + 1 || *last || !std::isfinite(hz) || hz < 0) return false;
		freqs.emplace_back(static_cast<int>(ch), hz);
		if (end == std::string::npos) return true;
		pos = end + 1;
		if (pos == spec.size()) return false;
	}
	return true;
}

int main(int argc, char **argv) {
	if (argc < 3 || argc > 6) {
		fprintf(stderr, "usage: %s <port> <logfile> [seconds; 0=until signal] [trig0_hz|freq=ch:hz,...] [status_file]\n", argv[0]);
		return 2;
	}
	const char *status_path = argc > 5 ? argv[5] : nullptr;
	const auto fail = [&](const char *reason) {
		fprintf(stderr, "%s\n", reason);
		writeStatus(status_path, "FAILED");
		return 1;
	};
	char *end = nullptr;
	const long secs = argc > 3 ? strtol(argv[3], &end, 10) : 5;
	if (secs < 0 || secs > INT_MAX || (argc > 3 && (end == argv[3] || *end))) return fail("invalid duration");
	std::vector<std::pair<int, double>> freqs;
	if (argc > 4 && !parseFreqs(argv[4], freqs)) return fail("invalid frequency specification");
	signal(SIGINT, [](int) { g_interrupted = 1; });
	signal(SIGTERM, [](int) { g_interrupted = 1; });
	SensorSyncSession s;
	const bool opened = s.open(argv[1]);
	if (!s.startupLog().empty()) {
		fprintf(stderr, "SensorSync pre-session synchronization (not capture data):\n");
		fwrite(s.startupLog().data(), 1, s.startupLog().size(), stderr);
	}
	if (!opened) {
		fprintf(stderr, "open/synchronize %s failed: %s\n", argv[1], s.lastError().c_str());
		return fail("cannot prepare recording port");
	}
	if (!s.start(argv[2], freqs)) {
		fprintf(stderr, "START failed: %s\n", s.lastError().c_str());
		return fail("cannot start timing session");
	}
	if (!writeStatus(status_path, "RUNNING")) {
		s.stop();
		return fail("cannot publish recorder status");
	}
	const auto start = std::chrono::steady_clock::now();
	while (!g_interrupted && s.ok() &&
	       (secs == 0 || std::chrono::steady_clock::now() - start < std::chrono::seconds(secs))) {
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
	s.stop();
	if (!s.ok()) return fail("recording failed: port, protocol, reported event loss, or log write error");
	if (!writeStatus(status_path, "DONE")) return fail("cannot publish final recorder status");
	fprintf(stderr, "done.\n");
	return 0;
}
#endif
