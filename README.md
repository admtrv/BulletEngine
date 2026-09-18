<p align="center">
    <img src="assets/images/BulletEngine.png" alt="BulletEngine Logo" width="350">
</p>

<p align="center">
  <strong>Powered by</strong>
</p>

<p align="center">

  <a href="https://github.com/admtrv/BulletPhysics">
    <img src="assets/images/BulletPhysics.png" alt="BulletPhysics Logo" height="70">
  </a>

  <a href="https://github.com/admtrv/BulletRender">
    <img src="assets/images/BulletRender.png" alt="BulletRender Logo" height="70">
  </a>
  
</p>

Small but complete C/C++ 3D game engine with Lua scripting. It combines a graphics engine module and a physics engine module under an ECS-based architecture, with an editor and its own project and scene workflow. The engine follows one core principle: provide a small, solid foundation for common needs, and leave the game-specific architecture to the programmer.

<p align="center">
    <img src="assets/images/Demo.gif" alt="Demo Editor" width="500">
</p>

## Features

- **Entities** live in a flat world and hold their components, transforms form parent-child hierarchy
- **Components** are pieces of behavior an entity is made of, attached and detached while world runs
- **Reflection** lets a component be described once, and editor and scene format pick it up on their own
- **Editor** shows the whole project, world is built and played without leaving it
- **Scripting** in Lua, small language that embeds into engine, this is where game itself is written
- **Interface** is drawn by scripts over the game, text and images placed in pixels
- **Events** carry what happened by name, so one script answers another without knowing it
- **Scenes** save as readable text, so they can be read and fixed by hand as well as by editor
- **Prefabs** keep an entity with everything below it, place it back anywhere
- **Assets** of any kind are handled the same way, and reload on the fly when file changes

## Dependencies

Modules come as submodules, everything they need applies here too:

- [BulletRender](https://github.com/admtrv/BulletRender) for graphics
- [BulletPhysics](https://github.com/admtrv/BulletPhysics) for physics

Bundled in `external/`:

- **Lua** and **sol2** for scripting

## Usage

Open a game project folder with engine:

```
BulletEngine ~/Games/NewGame
```

## Structure

```
src/
├── app/          app and scheduler
├── ecs/          entities, components, systems
├── script/       lua runtime and script api
├── scene/        reading and writing scenes and prefabs
├── assets/       registry and loaders
├── project/      project tree and its files
├── reflect/      runtime type description
├── interface/    editor panels
└── io/           logging
```

## Future Work

What comes next:

- **Audio**
- **Build**
- **2D/3D**
- **Animations**
- **Particles**
