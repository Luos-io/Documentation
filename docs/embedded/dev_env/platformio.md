# Setup a Luos PlatformIO project

> **Warning:** Make sure to read and understand [General intergration consideration](./general-integration-consideration.md) before reading this page.

<a href="https://platformio.org/" target="_blank">PlatformIO</a> is a cross-platform, cross-architecture, multiple framework, professional tool for embedded systems engineers and for software developers who write applications for embedded products. It can be used as a plug-in in a lot of <a href="https://docs.platformio.org/en/latest/integration/ide/index.html#desktop-ide" target="_blank">different editors</a>.

## Getting Started
 1. Install Platform IO on VSCode by following the instructions on <a href="https://platformio.org/platformio-ide" target="_blank">this page</a>.
 2. Create a new project on PlatformIO
 3. Add Luos as a dependancy and select HAL on your `platformio.ini` file:

```Json
lib_deps = Luos
board = <board name>
build_flags =
    -DLUOSHAL= <LuosHAL folder>
```
Replace `<board name>` with the name of the board you are using, eg. `board = l0` for the L0 board or `board = disco_f030r8` for the STM32 F0308 Discovery board

Replace `<LuosHAL folder>` with the <a href="https://github.com/Luos-io/LuosHAL" target="_blank">Luos HAL folder</a> corresponding to the family of the MCU you are usign, eg. `-DLUOSHAL=STM32F0` for the F0 family.

> *Note:* More information about how Luos libraries are managed into PlatformIO is available by <a href="https://community.luos.io/t/how-to-link-luos-with-platformio/303" target="\_blank">reading this post on our forum</a>.

### Project examples
Luos provides a lot of <a href="https://github.com/Luos-io/Examples/tree/master/Projects" target="_blank">code examples</a>, feel free to use and modify them as you want.

## Demonstration boards
Luos created [a sets of demonstration boards](../../demo_boards/boards-list.md) to allow you to easily test the technology.
