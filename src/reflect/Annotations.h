/*
 * Annotations.h
 */

#pragma once

#include <meta>
#include <string_view>

namespace BulletEngine {
namespace reflect {

// annotation has to be structural, so text is held as pointer

// what field is called

struct Name {
    consteval Name(std::string_view text) : text(std::define_static_string(text)) {}

    const char* text;        // what file calls field, when member is called otherwise
};

struct Label {
    consteval Label(std::string_view text) : text(std::define_static_string(text)) {}

    const char* text;        // what inspector draws, when name reads badly
};

struct Axes {
    consteval Axes(std::string_view text) : text(std::define_static_string(text)) {}

    const char* text;        // caption over row of three
};

// how value is edited

struct Range {
    float min;
    float max;
};

struct Speed {
    float value;
};

struct Asset {};
struct Color {};
struct Bits {};

// how field is laid out

struct Inline {};       // object hands its row to first field
struct Hidden {};       // out of inspector, still written to file

// read before field is made, so it never reaches metadata
struct Skip {};

} // namespace reflect
} // namespace BulletEngine
