/*
 * Log.cpp
 */

#include "Log.h"

#include <iostream>

namespace BulletEngine {
namespace io {

constexpr size_t MAX_TEXT = 64 * 1024;      // oldest lines go once the journal is full

Log::Sink::Sink(std::ostream& stream) : m_stream(stream)
{
    m_origin = stream.rdbuf(this);
}

Log::Sink::~Sink()
{
    m_stream.rdbuf(m_origin);
}

int Log::Sink::overflow(int c)
{
    if (c != traits_type::eof())
    {
        const char byte = static_cast<char>(c);
        write(&byte, 1);
    }

    return c;
}

std::streamsize Log::Sink::xsputn(const char* data, std::streamsize count)
{
    write(data, count);
    return count;
}

void Log::Sink::write(const char* data, std::streamsize count)
{
    m_origin->sputn(data, count);

    // a crash must not take the tail of the log with it
    m_origin->pubsync();

    Log::instance().append({data, static_cast<size_t>(count)});
}

Log& Log::instance()
{
    static Log log;
    return log;
}

Log::~Log()
{
    release();
}

void Log::capture()
{
    if (m_out)
    {
        return;
    }

    m_out = std::make_unique<Sink>(std::cout);
    m_err = std::make_unique<Sink>(std::cerr);
}

void Log::release()
{
    m_out.reset();
    m_err.reset();
}

void Log::append(std::string_view text)
{
    const std::lock_guard<std::mutex> lock(m_mutex);

    m_text += text;
    trim();

    m_revision++;
}

void Log::clear()
{
    const std::lock_guard<std::mutex> lock(m_mutex);

    m_text.clear();
    m_revision++;
}

std::string Log::getText() const
{
    const std::lock_guard<std::mutex> lock(m_mutex);
    return m_text;
}

uint32_t Log::getRevision() const
{
    const std::lock_guard<std::mutex> lock(m_mutex);
    return m_revision;
}

void Log::trim()
{
    if (m_text.size() <= MAX_TEXT)
    {
        return;
    }

    // cut on a line boundary, a half line would read as garbage
    const size_t excess = m_text.size() - MAX_TEXT;
    const size_t line = m_text.find('\n', excess);

    m_text.erase(0, line == std::string::npos ? m_text.size() : line + 1);
}

} // namespace io
} // namespace BulletEngine
