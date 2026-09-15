/*
 * Log.h
 */

#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <ostream>
#include <streambuf>
#include <string>
#include <string_view>

namespace BulletEngine {
namespace io {

// for now only a mirror of the standard streams, a real logger comes later
class Log {
public:
    static Log& instance();

    void capture();
    void release();

    void append(std::string_view text);
    void clear();

    std::string getText() const;

    // grows on every change, reader knows its copy went stale
    uint32_t getRevision() const;

private:
    Log() = default;
    ~Log();

    Log(const Log&) = delete;
    Log& operator=(const Log&) = delete;

    void trim();

    // what a stream writes goes to the journal and on to the terminal
    class Sink : public std::streambuf {
    public:
        explicit Sink(std::ostream& stream);
        ~Sink() override;

    protected:
        int overflow(int c) override;
        std::streamsize xsputn(const char* data, std::streamsize count) override;

    private:
        void write(const char* data, std::streamsize count);

        std::ostream& m_stream;
        std::streambuf* m_origin = nullptr;
    };

    mutable std::mutex m_mutex;
    std::string m_text;
    uint32_t m_revision = 0;

    std::unique_ptr<Sink> m_out;
    std::unique_ptr<Sink> m_err;
};

} // namespace io
} // namespace BulletEngine
