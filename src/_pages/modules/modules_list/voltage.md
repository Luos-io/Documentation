# Introduction to the Voltage module type

The Voltage module is generally a sensor measuring voltage.

The Voltage module type has access to all common capabilities.

----

## Functions

| **Function name and parameters** | **Action** | **Comment** |
| :---: | :---: | :---: |
| control(self) | Displays module type graphical interface | Only available using Jupyter notebook |

## Variables

| **Variable name** | **Action** | **Type** |
| :---: | :---: | :---: |
| volt | Reads or writes voltage in V | read / write: Float |
| threshold | Thresholds voltage variation before filter_changed event trigger. Default value 1.0 V. | read / write: Float |

## Events

| **Event name** | **Trigger** |
| :---: | :---: |
| changed | Any state modification falling or raising |
| filter_changed | Voltage variation bigger than *threshold* |

<div class="cust_edit_page"><a href="https://{{gh_path}}{{boards_path}}/voltage.md">Edit this page</a></div>
