# Angle module type

The Angle module handles a rotation position value in degree.

Its type has access to all common capabilities.

----

## Variables

| **Variable name** | **Action** | **Type** |
|:---:|:---:|:---:|
| rot_position | Reads the rotation position in degree | Read only: Float |
| threshold | Thresholds position variation before *filter_changed* event triggers. Default value 10 °. | Read / write: Float |

## Events

| **Event name** | **Trigger** |
|:---:|:---:|
| changed | Any movement on the position measurement |
| filter_changed | Movement bigger than *threshold* |

<div class="cust_edit_page"><a href="https://{{gh_path}}{{modules_path}}/angle.md">Edit this page</a></div>
