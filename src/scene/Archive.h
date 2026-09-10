/*
 * Archive.h
 */

#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <string>
#include <vector>

namespace BulletEngine {
namespace scene {

// text tree, one node per line, nesting by indent
class Node {
public:
    Node() = default;
    explicit Node(std::string name) : m_name(std::move(name)) {}

    const std::string& getName() const { return m_name; }
    void setName(std::string name) { m_name = std::move(name); }

    const std::string& getValue() const { return m_value; }
    void setValue(std::string value) { m_value = std::move(value); }

    bool hasValue() const { return !m_value.empty(); }

    Node& add(std::string name);
    const std::vector<Node>& getChildren() const { return m_children; }
    std::vector<Node>& getChildren() { return m_children; }

    const Node* find(std::string_view name) const;
    Node* find(std::string_view name);

    bool isEmpty() const { return m_children.empty() && m_value.empty(); }

private:
    std::string m_name;
    std::string m_value;
    std::vector<Node> m_children;
};

std::string toText(bool value);
std::string toText(int value);
std::string toText(float value);
std::string toText(const std::string& value);
std::string toText(const glm::vec2& value);
std::string toText(const glm::vec3& value);
std::string toText(const glm::vec4& value);
std::string toText(const glm::quat& value);

bool fromText(const std::string& text, bool& out);
bool fromText(const std::string& text, int& out);
bool fromText(const std::string& text, float& out);
bool fromText(const std::string& text, std::string& out);
bool fromText(const std::string& text, glm::vec2& out);
bool fromText(const std::string& text, glm::vec3& out);
bool fromText(const std::string& text, glm::vec4& out);
bool fromText(const std::string& text, glm::quat& out);

bool write(const Node& root, const std::string& path);
bool read(Node& root, const std::string& path);

std::string toString(const Node& root);
bool fromString(Node& root, const std::string& text);

} // namespace scene
} // namespace BulletEngine
