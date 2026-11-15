// session_client.cpp - host recorder for SensorSync-Logger
//
//   SensorSyncSession s;
//   s.open("/dev/serial/by-id/usb-Teensyduino_...");     // stops any running session
//   s.start("/data/run_0007.log", {{0, 25.0}, {2, 2.0}});  // channel -> Hz; 0/1 = FX pair, 2/3 = JAI pair
//   ... acquisition; poll s.ok() ...
//   s.stop();
//
// The reader thread copies every board byte into the log file.
// ok() turns false on a port/file error, a #ERR line, or #H drops != 0.
//
// Include this file in an application, or build the command-line recorder:
//   g++ -std=c++17 -O2 -pthread -DSESSION_CLIENT_DEMO session_client.cpp -o session_client
//   ./session_client <port> <log> [seconds, 0 = until Ctrl-C] [FX_hz | freq=FX:hz,JAI:hz]

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <poll.h>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>


class SensorSyncSession {
public:
	~SensorSyncSession() {
		stop();
		if (fd_ >= 0) {
			::close(fd_);
		}
	}

	// raw 115200 8N1 (USB CDC: nominal); STOP clears a session left running
	bool open(const char *port) {
		if (fd_ >= 0) {
			::close(fd_);
		}
		fd_ = ::open(port, O_RDWR | O_NOCTTY | O_CLOEXEC);
		if (fd_ < 0) {
			return fail(std::string("open ") + port, errno);
		}
		termios tio{};
		::tcgetattr(fd_, &tio);
		::cfmakeraw(&tio);
		::cfsetspeed(&tio, B115200);
		::tcsetattr(fd_, TCSANOW, &tio);
		send("\nSTOP\n");
		startup_log_ = readFor(std::chrono::milliseconds(500));	 // board replies, old session tail
		return true;
	}

	bool start(const std::string &path, const std::vector<std::pair<int, double>> &freqs = {}) {
		std::string spec;
		if (fd_ < 0) {
			return fail("port not open");
		}
		if (running_) {
			return fail("already recording");
		}
		if (!rateSpec(freqs, spec)) {
			return false;
		}
		out_ = std::fopen(path.c_str(), "wx");	// never overwrite a log
		if (!out_) {
			return fail("create " + path, errno);
		}
		ready_ = stopped_ = failed_ = false;
		::tcflush(fd_, TCIFLUSH);
		running_ = true;
		reader_ = std::thread(&SensorSyncSession::readLoop, this);
		send("START " + (spec.empty() ? "" : "freq=" + spec + " ") + path + "\n");
		if (!waitFor(ready_)) {
			if (!failed_) {
				fail("no #READY from board");
			}
			stop();
			return false;
		}
		return true;
	}

	void stop() {
		if (!running_) {
			return;
		}
		send("STOP\n");
		if (!waitFor(stopped_)) {
			fail("no #SESSION,STOP from board");
		}
		running_ = false;
		reader_.join();
		if (std::fflush(out_) != 0 || ::fsync(::fileno(out_)) != 0 || std::fclose(out_) != 0) {
			fail("write log", errno);
		}
		out_ = nullptr;
	}

	bool ok() const { return !failed_; }

	int lastErrno() const { return last_errno_; }

	std::string lastError() const {
		std::lock_guard<std::mutex> lock(mutex_);
		return last_error_;
	}

	const std::string &startupLog() const { return startup_log_; }	// replies to open(), not log data

private:
	bool fail(const std::string &why, int err = 0) {
		std::lock_guard<std::mutex> lock(mutex_);
		if (!failed_) {	 // keep the first cause
			last_errno_ = err;
			last_error_ = err ? why + ": " + std::strerror(err) : why;
		}
		failed_ = true;
		return false;
	}

	void send(const std::string &text) {
		if (::write(fd_, text.data(), text.size()) != ssize_t(text.size())) {
			fail("serial write", errno);
		}
	}

	// group rates on the wire; both channels of a pair must agree
	bool rateSpec(const std::vector<std::pair<int, double>> &freqs, std::string &spec) {
		double hz[2] = {};
		bool set[2] = {};
		for (const auto &[ch, f]: freqs) {
			if (ch < 0 || ch > 3) {
				return fail("trigger channel must be 0..3");
			}
			if (set[ch / 2] && hz[ch / 2] != f) {
				return fail("conflicting rates within a trigger pair");
			}
			hz[ch / 2] = f;
			set[ch / 2] = true;
		}
		for (int g = 0; g < 2; ++g)
			if (set[g]) {
				char item[48];
				std::snprintf(item, sizeof(item), "%s%s:%.17g", spec.empty() ? "" : ",", g ? "JAI" : "FX", hz[g]);
				spec += item;
			}
		return true;
	}

	std::string readFor(std::chrono::milliseconds span) {
		std::string text;
		char buf[4096];
		const auto end = std::chrono::steady_clock::now() + span;
		while (std::chrono::steady_clock::now() < end) {
			pollfd p{ fd_, POLLIN, 0 };
			if (::poll(&p, 1, 20) > 0) {
				const ssize_t n = ::read(fd_, buf, sizeof(buf));
				if (n > 0) {
					text.append(buf, n);
				}
			}
		}
		return text;
	}

	bool waitFor(const std::atomic<bool> &flag) {
		const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(3);
		while (!flag && !failed_ && std::chrono::steady_clock::now() < end) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		return flag;
	}

	void readLoop() {
		std::string line;
		char buf[4096];
		while (running_) {
			pollfd p{ fd_, POLLIN, 0 };
			if (::poll(&p, 1, 50) <= 0) {
				continue;
			}
			const ssize_t n = ::read(fd_, buf, sizeof(buf));
			if (n <= 0) {
				fail("serial port lost", n < 0 ? errno : 0);
				return;
			}
			if (std::fwrite(buf, 1, n, out_) != size_t(n)) {
				fail("write log", errno);
			}
			for (ssize_t i = 0; i < n; ++i) {
				if (buf[i] == '\n') {
					onLine(line);
					line.clear();
				} else if (buf[i] != '\r') {
					line += buf[i];
				}
			}
		}
	}

	void onLine(const std::string &line) {
		const auto starts = [&](const char *prefix) { return line.rfind(prefix, 0) == 0; };
		const std::string zero = " drops=0";
		if (line == "#READY") {
			ready_ = true;
		}
		else if (starts("#SESSION,STOP,")) {
			stopped_ = true;
		}
		else if (starts("#ERR")) {
			fail("board: " + line);
		}
		else if (starts("#H ") && (line.size() < zero.size() || line.compare(line.size() - zero.size(), zero.size(), zero) != 0)) {
			fail("board reported lost lines: " + line);
		}
	}

	int fd_ = -1;
	std::FILE *out_ = nullptr;
	std::thread reader_;
	std::atomic<bool> running_{ false }, ready_{ false }, stopped_{ false }, failed_{ false };
	mutable std::mutex mutex_;
	int last_errno_ = 0;
	std::string last_error_, startup_log_;
};


#ifdef SESSION_CLIENT_DEMO
	#include <csignal>
	#include <cstdlib>

static volatile std::sig_atomic_t g_stop = 0;

// "25" | "FX:25,JAI:2" | "freq=FX:25,JAI:2"
static std::vector<std::pair<int, double>> parseFreqs(std::string spec) {
	std::vector<std::pair<int, double>> freqs;
	if (spec.rfind("freq=", 0) == 0) {
		spec.erase(0, 5);
	}
	if (!spec.empty() && spec.find(':') == std::string::npos) {
		spec = "FX:" + spec;
	}
	for (size_t pos = 0; pos < spec.size();) {
		const size_t end = std::min(spec.find(',', pos), spec.size());
		const std::string item = spec.substr(pos, end - pos);
		const int ch = item.rfind("FX:", 0) == 0 ? 0 : item.rfind("JAI:", 0) == 0 ? 2 : -1;
		freqs.emplace_back(ch, std::atof(item.c_str() + item.find(':') + 1));
		pos = end + 1;
	}
	return freqs;
}

int main(int argc, char **argv) {
	if (argc < 3) {
		std::fprintf(stderr, "usage: %s <port> <log> [seconds, 0 = until Ctrl-C] [FX_hz | freq=FX:hz,JAI:hz]\n", argv[0]);
		return 2;
	}
	const int seconds = argc > 3 ? std::atoi(argv[3]) : 0;
	const auto freqs = parseFreqs(argc > 4 ? argv[4] : "");
	std::signal(SIGINT, [](int) { g_stop = 1; });
	std::signal(SIGTERM, [](int) { g_stop = 1; });

	SensorSyncSession s;
	if (!s.open(argv[1]) || !s.start(argv[2], freqs)) {
		std::fprintf(stderr, "start failed: %s\n", s.lastError().c_str());
		return 1;
	}
	std::fprintf(stderr, "recording -> %s\n", argv[2]);
	const auto t0 = std::chrono::steady_clock::now();
	while (!g_stop && s.ok() && (seconds == 0 || std::chrono::steady_clock::now() - t0 < std::chrono::seconds(seconds)))
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	s.stop();
	if (!s.ok()) {
		std::fprintf(stderr, "recording failed: %s\n", s.lastError().c_str());
		return 1;
	}
	std::fprintf(stderr, "done: %s\n", argv[2]);
	return 0;
}
#endif
