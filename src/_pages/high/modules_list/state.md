# State module type

The State module can handles a sensor (Button board for example), or an actuator (Power Switch board for example). Generally, this type of modules allows to manage bi-state elements such as on/off, pushed/release, 0/1, ...

Its type has access to all common capabilities.

----

## Functions

| **Function name and parameters** | **Action** | **Comment** |
|:---:|:---:|:---:|
| control(self) | Displays module type graphical interface | Only available using Jupyter notebook |

## Variables

| **Variable name** | **Action** | **Type** |
|:---:|:---:|:---:|
| state | Sets or reads the module state | read / write: Boolean (True or False) |

## Events

| **Event name** | **Trigger** |
|:---:|:---:|
| changed | Any state modification falling or raising |
| falling | State modification from True to False |
| raising | State modification from False to True |

<div class="cust_edit_page"><a href="https://{{gh_path}}{{modules_path}}/state.md">Edit this page</a></div>
