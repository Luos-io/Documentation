# luos-io.github.io
Luos documentation: [access to the full documentation!](http://doc.luos.io)

## Introduction
We started designing Luos with the conviction that building electronic systems should be made easier than it is today. Most of the time should be spent on designing the applications and behaviors instead of on complex and time-and-money-eating technicalities. To give a simple example, adding a new sensor —for instance a lidar— to an electronic device in conception should not take more than a few minutes. So you can try, test and iterate fast on a project to truly design what users want.

Today, Luos is exactly like a microservices architecture it encapsulates any software or hardware function to make it communicate and work with any other encapsulated module, however it was developed.

We have imagined Luos around three key concepts:

Luos Core: a tiny library that can be added to the MCU of an embedded board to make it seamlessly communicate with other boards. When conceiving electronic devices, you should be able to compose it from multiple functional parts and not think about communication issues between these parts.
Luos Driver: a hardware abstraction to make the integration of a sensor, an effector or a function as easy at it should be —that is, with one line of code! Drivers must use standardized APIs to let you switch from one type of sensors to another without breaking anything. For example, all types of distance sensors should return a distance in meters using a same function `get_distance()`.
Robus: A communication bus dedicated to embedded system communication, to make all the above natural and simple.

[Access to the full documentation!](http://doc.luos.io)

## mdBook
We use the static page generator mdBook for this documentation.
To use and visualize it locally:

 - Download and install [Rust](https://www.rust-lang.org/) (at least 1.35) and Cargo
 - Install mdBook: `cargo install mdbook`
 - In the doc folder, run `mdbook serve` to serves it at `http://localhost:3000`

More information: <a href="https://rust-lang.github.io/mdBook/index.html" target="_blank">Official mdBook documentation</a>
