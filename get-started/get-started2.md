---
custom_edit_url: null
---

# Part 2: Take the control

## The remote control part: you can control the Matrix 💊

The gate running on your board allows you to take control of any service loaded on your device.

### Setup development environment

We will use Python with the default library of Luos called pyluos.
To install it, run:

```bash
pip install pyluos
```

This step needs you to have <a href="IPython" target="_blank">IPython &#8599;</a> previously installed on your computer.

### Connect and control your device

Pyluos provides a set of tools. To control you device, run:

```bash
pyluos-shell
```

This command will find the serial port of your device and mount it into a "device" object.

For example:

```bash
$ pyluos-shell
Searching for a gate available
Testing /dev/cu.usbserial-D308N885
Testing /dev/cu.usbmodem13102
Connected to "/dev/cu.usbmodem13102".
Sending detection signal.
Waiting for routing table...
Device setup.

 Hit Ctrl-D to exit this interpreter.

Your luos device have been successfully mounted into a "device" object:
  ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
  ┃  ╭node 1            /!\ Not certified            ┃
  ┃  │  Type                Alias               ID   ┃
  ┃  ├> State               led                 2    ┃
  ┃  ├> Pipe                Pipe                3    ┃
  ┃  ├> Gate                gate                1    ┃
  ┃  ╰> Unknown             blinker             4    ┃
╔>┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛

```

Now that you are on an IPython command line, you can run Python scripts in it.
The `device` object is your real device and you can interact with it. For example, try to execute these lines one by one:

`device.blinker.time=0.25`  
`device.blinker.pause()`  
`device.led.state=True`  
`device.led.state=False`  
`device.blinker.play()`

## Next steps

Your development environment is now installed and you have a Luos app running on your MCU. The [next part](/get-started/get-started3) of this section deals with creating your first Luos network.
