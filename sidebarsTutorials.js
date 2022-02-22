module.exports = {
  documentation: [
    {
      type: 'category',
      label: 'Tutorials',
      items: [
        {
          type: 'doc',
          label: 'Tutorials',
          id: 'tutorials',
        },
        {
          type: 'category',
          label: 'Luos and tools',
          items: [
            {
              type: 'doc',
              label: 'List',
              id: 'luos-and-tools/list',
            },
            {
              type: 'doc',
              label: 'Bootloader',
              id: 'luos-and-tools/bootloader',
            },
            {
              type: 'doc',
              label: 'Use Luos with Arduino IDE',
              id: 'luos-and-tools/arduino',
            },
            {
              type: 'doc',
              label: 'FreeRTOS',
              id: 'luos-and-tools/freertos',
            },
          ],
        },
        {
          type: 'category',
          label: 'Luos integration',
          items: [
            {
              type: 'doc',
              label: 'List',
              id: 'luos-integration/list',
            },
            {
              type: 'doc',
              label: 'Install ROS 2',
              id: 'luos-integration/install-ros2',
            },
            {
              type: 'doc',
              label: 'ROS 2 package example',
              id: 'luos-integration/ros2-package-example',
            },
            {
              type: 'doc',
              label: 'ROS 1 retro-compatibility with Luos',
              id: 'luos-integration/ros1-retrocompatibility',
            },
          ],
        },
      ],
    },
    { type: 'link', label: 'Go to Luos.io', href: 'https://www.luos.io/' },
  ],
};
