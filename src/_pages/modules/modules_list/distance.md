# Distance module type

The Distance module handles a sensor measuring a distance in mm.

Its type has access to all common capabilities.

----

## Variables

| **Variable name** | **Action** | **Type** |
|:---:|:---:|:---:|
| distance | Reads the measured distance in mm | read only: Float |
| threshold | Thresholds distance variation before filter_changed event trigers. Default value 10 mm. | read / write: Float |

## Events

| **Event name** | **Trigger** |
|:---:|:---:|
| changed | Any movement on the distance measurement |
| filter_changed | Movement bigger than *threshold* |

<div class="cust_edit_page"><a href="https://{{gh_path}}{{modules_path}}/distance.md">Edit this page</a></div>
