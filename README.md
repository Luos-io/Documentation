[![](https://img.shields.io/github/license/Luos-io/Documentation)](https://github.com/Luos-io/Documentation/blob/master/LICENSE)

# doc.luos.io
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

[![](https://img.shields.io/discourse/topics?server=https%3A%2F%2Fcommunity.luos.io&logo=Discourse)](https://community.luos.io)
[![](https://img.shields.io/badge/Doc-Here-34A3B4?style=flat&logo=data%3Aimage%2Fpng%3Bbase64%2CiVBORw0KGgoAAAANSUhEUgAAACAAAAASCAYAAAA6yNxSAAABhGlDQ1BJQ0MgcHJvZmlsZQAAKJF9kT1Iw0AcxV9TpSIVQTsUdchQnSyIijhKFYtgobQVWnUwufQLmjQkKS6OgmvBwY%2FFqoOLs64OroIg%2BAHi5Oik6CIl%2Fi8ptIjx4Lgf7%2B497t4BQqPCVLNrAlA1y0jFY2I2tyoGXuHHMAYgICwxU0%2BkFzPwHF%2F38PH1LsqzvM%2F9OfqUvMkAn0g8x3TDIt4gntm0dM77xCFWkhTic%2BJxgy5I%2FMh12eU3zkWHBZ4ZMjKpeeIQsVjsYLmDWclQiaeJI4qqUb6QdVnhvMVZrdRY6578hcG8tpLmOs0RxLGEBJIQIaOGMiqwEKVVI8VEivZjHv4hx58kl0yuMhg5FlCFCsnxg%2F%2FB727NwtSkmxSMAd0vtv0xCgR2gWbdtr%2BPbbt5AvifgSut7a82gNlP0uttLXIE9G8DF9dtTd4DLneA8JMuGZIj%2BWkKhQLwfkbflAMGb4HeNbe31j5OH4AMdbV8AxwcAmNFyl73eHdPZ2%2F%2Fnmn19wNDWHKUFMvAtAAAAAZiS0dEAAAAAAAA%2BUO7fwAAAAlwSFlzAAALEgAACxIB0t1%2B%2FAAAAAd0SU1FB%2BQDGxIvCxvVz4EAAAAZdEVYdENvbW1lbnQAQ3JlYXRlZCB3aXRoIEdJTVBXgQ4XAAACSElEQVRIx8WVXWiOYRjHf8%2BMxnzMCZt8nWhxtBpq0VptB9QOFQfYAUXNIaUkBysHUpyIIwc%2B4mBSpleUtDlRbEWk5G3FYlKzJotp78%2BB69HT693eNPVedXffz%2FW%2Frvv%2Bd1%2F%2F%2B3pQq9Xl6iIqYWq7v%2B10Jc6vAqZjPVUpAhWz1tZWqsuU5886SZIZsbBaoB6oASaBEeBnkiRp7AJgHTAf%2BACM53I5UNtCA6dKENihXlWbS2AH1Stqg9pjadujom5Sp4qw3ersNwA0AnuBW8BgEdYCLAVyQBMwBlwHXgNtQCdwA%2BgAtgLzgGvAC%2BAEcBN4Xj2HEn6Ja28CLgBH0jKpFyPmGXAAOBNEj0dJekN%2F%2BSyBpMQhySzYA%2BA%2BMJA9PKsXdTvwCegG9gOJapIkw%2F%2FjFWyO%2BVyxQDNEvgOX4qbao0ydWQHPhUBjzC%2FLxA3FXAPUAX3ApNpSTMBSL3EWLC3fZBkCadwgsAw4CywEBtSNVZnNa%2F5RA2leoVy%2FifkVMAEcAw4FsX1V8WwAmks0l%2FGY12SdhUIBYH187iqRl4qwHugCPgL9SZKkAn0YIauIRvEomkN3fKdjcfjzRf4mdVp9qj5RH6dY2iXVler7yO9Rd2byD4f%2FaJrQoI6Fc0IdjZFXz4d%2FNLrinUxX26Lei%2FU39a56We0PgqrH1RH1h9qr9gU2odZlGdeqJ9Uh9W2MvHo7Nsnauzg8ze1SPxfFDKnbAu8Igqm9UTf8Ja6ZapmxtcDX6ILZhpPiS4AVwHAqzszPCGB1vJqxFPsFVSEoxB5J85wAAAAASUVORK5CYII%3D)](https://doc.luos.io)
[![](https://img.shields.io/badge/LinkedIn-Follow%20us-0077B5?style=flat&logo=linkedin)](https://www.linkedin.com/company/luos)
