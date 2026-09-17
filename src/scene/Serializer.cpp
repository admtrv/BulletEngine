/*
 * Serializer.cpp
 */

#include "Serializer.h"

#include "Version.h"
#include "ecs/Ecs.h"
#include "reflect/Registry.h"

#include <typeindex>

namespace BulletEngine {
namespace scene {

constexpr const char* VERSION_NODE = "version";
constexpr const char* ENTITY_NODE = "entity";
constexpr const char* PREFAB_NODE = "prefab";

static void saveValue(Node& node, const reflect::Field& field, const void* instance)
{
    const reflect::Value value = field.get(instance);

    switch (field.getType())
    {
        case reflect::ValueType::Bool:   node.setValue(toText(value.get<bool>())); break;
        case reflect::ValueType::Int:    node.setValue(toText(value.get<int>())); break;
        case reflect::ValueType::Float:  node.setValue(toText(value.get<float>())); break;
        case reflect::ValueType::String: node.setValue(toText(value.get<std::string>())); break;
        case reflect::ValueType::Vec2:   node.setValue(toText(value.get<glm::vec2>())); break;
        case reflect::ValueType::Vec3:   node.setValue(toText(value.get<glm::vec3>())); break;
        case reflect::ValueType::Vec4:   node.setValue(toText(value.get<glm::vec4>())); break;
        case reflect::ValueType::Quat:   node.setValue(toText(value.get<glm::quat>())); break;
        default: break;
    }
}

static void loadValue(const Node& node, const reflect::Field& field, void* instance)
{
    const std::string& text = node.getValue();

    switch (field.getType())
    {
        case reflect::ValueType::Bool:   { bool v{};       if (fromText(text, v)) field.set(instance, v); break; }
        case reflect::ValueType::Int:    { int v{};        if (fromText(text, v)) field.set(instance, v); break; }
        case reflect::ValueType::Float:  { float v{};      if (fromText(text, v)) field.set(instance, v); break; }
        case reflect::ValueType::String: { std::string v;  if (fromText(text, v)) field.set(instance, v); break; }
        case reflect::ValueType::Vec2:   { glm::vec2 v{};  if (fromText(text, v)) field.set(instance, v); break; }
        case reflect::ValueType::Vec3:   { glm::vec3 v{};  if (fromText(text, v)) field.set(instance, v); break; }
        case reflect::ValueType::Vec4:   { glm::vec4 v{};  if (fromText(text, v)) field.set(instance, v); break; }
        case reflect::ValueType::Quat:   { glm::quat v{1.0f, 0.0f, 0.0f, 0.0f}; if (fromText(text, v)) field.set(instance, v); break; }
        default: break;
    }
}

void saveObject(Node& node, const reflect::Type& type, const void* instance)
{
    for (const reflect::Field* field : type.getAllFields())
    {
        Node& child = node.add(field->getName());

        if (field->getKind() == reflect::FieldKind::Object)
        {
            const reflect::Type* nested = nullptr;
            const void* object = field->resolve(instance, &nested);

            // concrete type on the line
            if (nested && object)
            {
                child.setValue(nested->getName());
                saveObject(child, *nested, object);
            }

            continue;
        }

        saveValue(child, *field, instance);
    }
}

void loadObject(const Node& node, const reflect::Type& type, void* instance)
{
    for (const reflect::Field* field : type.getAllFields())
    {
        const Node* child = node.find(field->getName());

        if (!child)
        {
            continue;
        }

        if (field->getKind() == reflect::FieldKind::Object)
        {
            const reflect::Type* nested = nullptr;
            void* object = field->resolve(instance, &nested);

            // empty slot built from the type named on the line
            if (!object && field->isBuildable())
            {
                nested = reflect::Registry::instance().find(child->getValue());
                object = nested ? field->build(instance, *nested) : nullptr;
            }

            if (nested && object)
            {
                loadObject(*child, *nested, object);
            }

            continue;
        }

        loadValue(*child, *field, instance);
    }
}

void saveEntity(Node& node, const ecs::World& world, ecs::Entity entity)
{
    node.setValue(std::to_string(entity));

    for (const std::unique_ptr<ecs::Component>& component : world.getComponents(entity))
    {
        const reflect::Type* type = reflect::Registry::instance().find(std::type_index(typeid(*component)));

        if (!type)
        {
            continue;
        }

        saveObject(node.add(type->getName()), *type, component.get());
    }
}

ecs::Entity loadEntity(ecs::World& world, const Node& node)
{
    const ecs::Entity entity = world.create();

    for (const Node& componentNode : node.getChildren())
    {
        const reflect::Type* type = reflect::Registry::instance().find(componentNode.getName());

        if (!type)
        {
            continue;
        }

        auto* component = static_cast<ecs::Component*>(type->create());

        if (!component)
        {
            continue;
        }

        loadObject(componentNode, *type, component);
        world.attach(entity, std::unique_ptr<ecs::Component>(component));
    }

    return entity;
}

Node toNode(const ecs::World& world)
{
    Node root;
    root.add(VERSION_NODE).setValue(VERSION);

    for (ecs::Entity entity : world.getEntities())
    {
        saveEntity(root.add(ENTITY_NODE), world, entity);
    }

    return root;
}

void fromNode(ecs::World& world, const Node& root)
{
    for (const Node& entityNode : root.getChildren())
    {
        if (entityNode.getName() == ENTITY_NODE)
        {
            loadEntity(world, entityNode);
        }
    }
}

bool savePrefab(const ecs::World& world, ecs::Entity entity, const std::string& path)
{
    Node root;
    root.add(VERSION_NODE).setValue(VERSION);

    saveEntity(root.add(PREFAB_NODE), world, entity);
    return write(root, path);
}

ecs::Entity loadPrefab(ecs::World& world, const std::string& path)
{
    Node root;

    if (!read(root, path))
    {
        return ecs::INVALID_ENTITY;
    }

    for (const Node& node : root.getChildren())
    {
        if (node.getName() == PREFAB_NODE)
        {
            return loadEntity(world, node);
        }
    }

    return ecs::INVALID_ENTITY;
}

ecs::Entity clone(ecs::World& world, ecs::Entity entity)
{
    // through tree, so copy goes same way file does
    Node node;
    saveEntity(node, world, entity);

    return loadEntity(world, node);
}

bool save(const ecs::World& world, const std::string& path)
{
    return write(toNode(world), path);
}

bool load(ecs::World& world, const std::string& path)
{
    Node root;

    if (!read(root, path))
    {
        return false;
    }

    fromNode(world, root);
    return true;
}

} // namespace scene
} // namespace BulletEngine
