/*
 * Components.cpp
 */

#include "Components.h"

#include "assets/Loaders.h"
#include "assets/Registry.h"
#include "project/Project.h"
#include "scene/Serializer.h"

#include "render/textures/CubeMapLoader.h"

#include <filesystem>
#include <utility>

#include <array>

#include "collision/collider/BoxCollider.h"
#include "collision/collider/CylinderCollider.h"
#include "collision/collider/GroundCollider.h"
#include "collision/collider/SphereCollider.h"

using namespace BulletEngine::ecs;
using namespace BulletPhysics::collision;
using namespace BulletPhysics::collision::collider;
// scene namespace is left out, its Mesh is gpu geometry rather than what entity shows
using BulletRender::scene::Light;
using BulletRender::scene::AmbientLight;
using BulletRender::scene::DirectionalLight;
using BulletRender::scene::PointLight;
using BulletRender::scene::SpotLight;

// empty key clears slot, key that fails to load leaves what is there
template<class T>
static bool reload(BulletEngine::assets::Handle<T>& handle, const std::string& key)
{
    if (key.empty())
    {
        handle.reset();
        return true;
    }

    handle = BulletEngine::assets::Registry::instance().load<T>(key);

    return handle.getState() != BulletEngine::assets::State::Failed;
}

void ScriptComponent::setScriptKey(const std::string& key)
{
    reload(script, key);
}

// environment

void EnvironmentComponent::setLayout(int layout)
{
    m_layout = layout == int(Layout::Faces) ? Layout::Faces : Layout::Cross;
}

void EnvironmentComponent::setCrossKey(const std::string& key)
{
    reload(m_cross, key);
}

void EnvironmentComponent::setFaceKey(int face, const std::string& key)
{
    reload(m_facePixels[face], key);

    m_faces.reset();    // no longer stands for what faces say
}

std::shared_ptr<BulletRender::render::CubeMap> EnvironmentComponent::buildFaces() const
{
    std::array<const BulletRender::render::TexturePixels*, FACE_COUNT> pictures;

    for (int face = 0; face < FACE_COUNT; face++)
    {
        pictures[face] = m_facePixels[face].get();

        if (!pictures[face])
        {
            return nullptr;
        }
    }

    return BulletRender::render::CubeMapLoader::uploadFaces(pictures);
}

// sky covers whole view, so what lies under it is seen only until it arrives
glm::vec3 EnvironmentComponent::getClearColor() const
{
    return isSkybox() && getSkybox() ? glm::vec3(0.0f) : m_color;
}

std::shared_ptr<BulletRender::render::CubeMap> EnvironmentComponent::getSkybox() const
{
    if (isCross())
    {
        return m_cross.getShared();
    }

    if (!isFaces())
    {
        return nullptr;
    }

    if (!m_faces)
    {
        m_faces = buildFaces();
    }

    return m_faces;
}

// material

void MaterialSlot::setTextureKey(const std::string& key)
{
    reload(m_texture, key);
}

BulletRender::render::TextureSlot MaterialSlot::get() const
{
    BulletRender::render::TextureSlot slot = m_slot;
    slot.texture = m_texture.getShared();

    return slot;
}

// only what the file names is taken, an empty entry leaves its slot as it stands
void Material::importFrom(const std::string& mtlPath)
{
    const project::Project& project = project::Project::instance();
    const std::vector<BulletRender::render::MaterialImport> imported = BulletRender::render::readMtl(mtlPath);

    if (imported.empty())
    {
        return;
    }

    const BulletRender::render::MaterialImport& source = imported.front();

    diffuse.color = source.diffuse;
    specular.color = source.specular;
    specular.shininess = source.shininess;
    emissive.color = source.emissive;

    const std::pair<MaterialSlot*, const std::string*> slots[] = {
        {&diffuse.texture, &source.diffuseTexture},
        {&specular.texture, &source.specularTexture},
        {&normal.texture, &source.normalTexture},
        {&emissive.texture, &source.emissiveTexture}
    };

    for (const auto& [slot, path] : slots)
    {
        // file naming no texture says nothing about slot, it does not empty it
        if (!path->empty())
        {
            slot->setTextureKey(project.getKey(*path));
        }
    }
}

// what the component holds, handed over as the renderer reads it
void Material::fill(BulletRender::render::Material& material) const
{
    material.shading = shading;

    material.diffuse = diffuse.color;
    material.diffuseTexture = diffuse.texture.get();

    material.specular = specular.color;
    material.specularTexture = specular.texture.get();
    material.shininess = specular.shininess;

    material.normalTexture = normal.texture.get();

    material.emissive = emissive.color;
    material.emissiveTexture = emissive.texture.get();

    material.alphaMode = settings.alphaMode;
    material.alphaCutoff = settings.alphaCutoff;
    material.doubleSided = settings.doubleSided;
}

// renderables

void Mesh::setModelKey(const std::string& key)
{
    if (!reload(m_model, key))
    {
        return;
    }

    // wavefront keeps the look beside the shape, taken once so the fields below hold it all
    std::filesystem::path path = project::Project::instance().getPath(key);

    if (path.extension() == ".obj")
    {
        material.importFrom(path.replace_extension(".mtl").string());
    }
}

Sprite::Sprite()
{
    // flat picture carries own shading, and holes in it are point of it
    material.shading = BulletRender::render::Shading::Unlit;
    material.settings.alphaMode = BulletRender::render::AlphaMode::Blend;

    // quad carries model uv, a picture read over it hangs upside down without this
    material.diffuse.texture.setFlippedV(true);

    setShape(int(Shape::Quad));
}

void Sprite::setShape(int kind)
{
    m_shapeType = kind == int(Shape::Circle) ? Shape::Circle : Shape::Quad;

    fitFrame();
}

void Sprite::setSource(int source)
{
    m_source = source == int(Source::Sheet) ? Source::Sheet : Source::Single;

    fitFrame();
}

void Sprite::setFrames(const glm::ivec2& frames)
{
    m_frames = glm::max(frames, glm::ivec2(1));

    fitFrame();
}

void Sprite::setFrame(int frame)
{
    const int count = m_frames.x * m_frames.y;

    // reading runs along the sheet, so counting past its end wraps to the start
    m_frame = count > 0 ? ((frame % count) + count) % count : 0;
}

glm::vec2 Sprite::getSheetSize() const
{
    const assets::Handle<BulletRender::render::Texture2D>& texture = material.diffuse.texture.getTexture();

    return texture ? glm::vec2(float(texture->getWidth()), float(texture->getHeight())) : glm::vec2(0.0f);
}

glm::vec2 Sprite::getFrameSize() const
{
    const glm::vec2 cells = isSheet() ? glm::vec2(m_frames) : glm::vec2(1.0f);

    return getSheetSize() / cells;
}

glm::vec2 Sprite::getFrameScale() const
{
    return isSheet() ? glm::vec2(1.0f) / glm::vec2(m_frames) : glm::vec2(1.0f);
}

glm::vec2 Sprite::getFrameOffset() const
{
    if (!isSheet())
    {
        return glm::vec2(0.0f);
    }

    // sheet is read left to right, top down, while the flipped uv climbs from the bottom
    const glm::ivec2 cell{m_frame % m_frames.x, m_frames.y - 1 - m_frame / m_frames.x};

    return glm::vec2(cell) * getFrameScale();
}

// called every frame, so rebuilds nothing until what it is cut from moves
void Sprite::fitFrame()
{
    const glm::vec2 frame = getFrameSize();

    if (frame == m_fitted && m_shapeType == m_fittedShape && m_shape)
    {
        return;
    }

    m_fitted = frame;
    m_fittedShape = m_shapeType;

    // circle stays round whatever picture is, only quad takes its proportions
    if (m_shapeType == Shape::Circle)
    {
        m_shape = assets::Registry::instance().load<BulletRender::scene::Model>(assets::CIRCLE_KEY);
        return;
    }

    // square keeps shape valid while picture is not there
    if (frame.x <= 0.0f || frame.y <= 0.0f)
    {
        m_shape = assets::Registry::instance().load<BulletRender::scene::Model>(assets::QUAD_KEY);
        return;
    }

    // longer side stays one unit, so sprite takes room square one would
    const float longest = std::max(frame.x, frame.y);

    m_shape = assets::Registry::instance().load<BulletRender::scene::Model>(assets::quadKey(frame / longest));
}
