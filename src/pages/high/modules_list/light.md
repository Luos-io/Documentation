# Light module type

The Light module handles a sensor measuring a light intensity in lux.

Its type has access to all common capabilities.

----

## Variables

| **Variable name** | **Action** | **Type** |
|:---:|:---:|:---:|
| lux | Reads the measured light intensity in lux | read only: Float |
| threshold | Thresholds light intensity variation before *filter_changed* event trigers. Default value 10 lux. | read / write: Float |

## Events

| **Event name** | **Trigger** |
|:---:|:---:|
| changed | Any movement on the light intensity measurement |
| filter_changed | Movement bigger than *threshold* |

<div class="cust_edit_page"><a href="https://{{gh_path}}{{modules_path}}/light.md">Edit this page</a></div>
