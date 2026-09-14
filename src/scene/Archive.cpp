/*
 * Archive.cpp
 */

#include "Archive.h"

#include <algorithm>
#include <charconv>
#include <fstream>
#include <sstream>

namespace BulletEngine {
namespace scene {

constexpr const char* INDENT = "  ";

static std::string_view trimRight(std::string_view text)
{
    const size_t end = text.find_last_not_of(" \t\r");
    return end == std::string_view::npos ? std::string_view{} : text.substr(0, end + 1);
}

static size_t countIndent(std::string_view line)
{
    const size_t first = line.find_first_not_of(' ');
    return first == std::string_view::npos ? 0 : first / 2;
}

// splits "1, 2, 3" into its numbers
static std::vector<float> parseNumbers(const std::string& text, size_t expected)
{
    std::vector<float> numbers;
    std::stringstream stream(text);
    std::string piece;

    while (std::getline(stream, piece, ','))
    {
        numbers.push_back(std::strtof(piece.c_str(), nullptr));
    }

    return numbers.size() == expected ? numbers : std::vector<float>{};
}

static std::string joinNumbers(const float* values, size_t count)
{
    std::ostringstream stream;

    for (size_t i = 0; i < count; i++)
    {
        stream << (i ? ", " : "") << values[i];
    }

    return stream.str();
}

static void writeNode(std::ostringstream& stream, const Node& node, int depth)
{
    for (int i = 0; i < depth; i++)
    {
        stream << INDENT;
    }

    stream << node.getName();

    if (node.hasValue())
    {
        stream << ": " << node.getValue();
    }

    stream << '\n';

    for (const Node& child : node.getChildren())
    {
        writeNode(stream, child, depth + 1);
    }
}

Node& Node::add(std::string name)
{
    m_children.emplace_back(std::move(name));
    return m_children.back();
}

const Node* Node::find(std::string_view name) const
{
    const auto it = std::find_if(m_children.begin(), m_children.end(),
        [name](const Node& child) { return child.getName() == name; });

    return it != m_children.end() ? &(*it) : nullptr;
}

Node* Node::find(std::string_view name)
{
    return const_cast<Node*>(static_cast<const Node*>(this)->find(name));
}

// text form

std::string toText(bool value) { return value ? "true" : "false"; }
std::string toText(int value) { return std::to_string(value); }

std::string toText(float value)
{
    std::ostringstream stream;
    stream << value;
    return stream.str();
}

std::string toText(const std::string& value) { return value; }
std::string toText(const glm::vec2& value) { return joinNumbers(&value.x, 2); }
std::string toText(const glm::vec3& value) { return joinNumbers(&value.x, 3); }
std::string toText(const glm::vec4& value) { return joinNumbers(&value.x, 4); }

std::string toText(const glm::quat& value)
{
    const float parts[4] = {value.w, value.x, value.y, value.z};
    return joinNumbers(parts, 4);
}

// value back from text

bool fromText(const std::string& text, bool& out)
{
    out = (text == "true" || text == "1");
    return text == "true" || text == "false" || text == "1" || text == "0";
}

bool fromText(const std::string& text, int& out)
{
    const char* begin = text.c_str();
    char* end = nullptr;
    const long parsed = std::strtol(begin, &end, 10);

    if (end == begin)
    {
        return false;
    }

    out = static_cast<int>(parsed);
    return true;
}

bool fromText(const std::string& text, float& out)
{
    const char* begin = text.c_str();
    char* end = nullptr;
    const float parsed = std::strtof(begin, &end);

    if (end == begin)
    {
        return false;
    }

    out = parsed;
    return true;
}

bool fromText(const std::string& text, std::string& out)
{
    out = text;
    return true;
}

bool fromText(const std::string& text, glm::vec2& out)
{
    const auto numbers = parseNumbers(text, 2);
    if (numbers.empty()) return false;

    out = {numbers[0], numbers[1]};
    return true;
}

bool fromText(const std::string& text, glm::vec3& out)
{
    const auto numbers = parseNumbers(text, 3);
    if (numbers.empty()) return false;

    out = {numbers[0], numbers[1], numbers[2]};
    return true;
}

bool fromText(const std::string& text, glm::vec4& out)
{
    const auto numbers = parseNumbers(text, 4);
    if (numbers.empty()) return false;

    out = {numbers[0], numbers[1], numbers[2], numbers[3]};
    return true;
}

bool fromText(const std::string& text, glm::quat& out)
{
    const auto numbers = parseNumbers(text, 4);
    if (numbers.empty()) return false;

    out = glm::quat(numbers[0], numbers[1], numbers[2], numbers[3]);
    return true;
}

// tree to text and back

std::string toString(const Node& root)
{
    std::ostringstream stream;

    for (const Node& child : root.getChildren())
    {
        writeNode(stream, child, 0);
    }

    return stream.str();
}

bool fromString(Node& root, const std::string& text)
{
    root = Node{};

    // path from root down, as child indices
    std::vector<size_t> stack;

    std::istringstream stream(text);
    std::string line;

    while (std::getline(stream, line))
    {
        const std::string_view trimmed = trimRight(line);

        if (trimmed.empty())
        {
            continue;
        }

        const size_t depth = countIndent(trimmed);

        if (depth > stack.size())
        {
            return false;
        }

        stack.resize(depth);

        Node* parent = &root;
        for (size_t index : stack)
        {
            parent = &parent->getChildren()[index];
        }

        const std::string_view content = trimmed.substr(depth * 2);
        const size_t colon = content.find(':');

        Node& node = parent->add(std::string(content.substr(0, colon)));

        if (colon != std::string_view::npos)
        {
            std::string_view value = content.substr(colon + 1);

            if (!value.empty() && value.front() == ' ')
            {
                value.remove_prefix(1);
            }

            node.setValue(std::string(value));
        }

        stack.push_back(parent->getChildren().size() - 1);
    }

    return true;
}

bool write(const Node& root, const std::string& path)
{
    std::ofstream file(path);

    if (!file)
    {
        return false;
    }

    file << toString(root);
    return file.good();
}

bool read(Node& root, const std::string& path)
{
    std::ifstream file(path);

    if (!file)
    {
        return false;
    }

    std::ostringstream buffer;
    buffer << file.rdbuf();

    return fromString(root, buffer.str());
}

} // namespace scene
} // namespace BulletEngine
