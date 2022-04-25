import React from 'react';
import styles from './anatomy.module.css';
import Grid from '@mui/material/Grid';
import List from '@mui/material/List';
import ListItem from '@mui/material/ListItem';
import ListItemText from '@mui/material/ListItemText';
import ListItemAvatar from '@mui/material/ListItemAvatar';
import Avatar from '@mui/material/Avatar';

const Anatomy = (props) => {
  return (
    <div className={styles.container}>
      <Grid container mt={3}>
        <Grid item md={6} lg={6} pl={7} pr={3}>
          <h1 className={`${styles.title} ${styles.underline}`}>
            Anatomy of Luos
          </h1>
          <p className={styles.text}>
            Luos gives you access to multiple features developed by developers
            for developers. Some of the features come from community needs.
          </p>
          <List sx={{ width: '100%' }}>
            <ListItem alignItems="flex-start" sx={{ marginBottom: '50px' }}>
              <ListItemAvatar>
                <Avatar
                  alt="table tree icon"
                  src="/img/index/anatomy/table-tree.svg"
                  variant="square"
                />
              </ListItemAvatar>
              <ListItemText
                primary="Topology and routing table"
                secondary={
                  'Luos detects every services in your system and locates them. Allowing you to access and adapt to any feature from anywhere.'
                }
              />
            </ListItem>

            <ListItem alignItems="flex-start" sx={{ marginBottom: '50px' }}>
              <ListItemAvatar>
                <Avatar
                  alt="cubes icon"
                  src="/img/index/anatomy/cubes.svg"
                  variant="square"
                />
              </ListItemAvatar>
              <ListItemText
                primary="Microservice architecture"
                secondary={
                  'Luos follows the microservice philosophy. Luos Engine is a library allowing you to think your features into small independant and loosly coupled bricks.'
                }
              />
            </ListItem>

            <ListItem alignItems="flex-start">
              <ListItemAvatar>
                <Avatar
                  alt="files icon"
                  src="/img/index/anatomy/files.svg"
                  variant="square"
                />
              </ListItemAvatar>
              <ListItemText
                primary="Remote control"
                secondary={
                  'You can access the topology and the routing table anywhere, even your computer, another machine or a cloud application.'
                }
              />
            </ListItem>
          </List>
        </Grid>
        <Grid item md={6} lg={6}>
          <img src="/img/index/anatomy-1.svg" />
        </Grid>
      </Grid>

      <Grid container mt={3}>
        <Grid item md={6} lg={6}>
          <img src="/img/index/anatomy-2.svg" />
        </Grid>
        <Grid item md={6} lg={6} pl={3} pr={7}>
          <List sx={{ width: '100%', marginTop: '50px' }}>
            <ListItem alignItems="flex-start" sx={{ marginBottom: '50px' }}>
              <ListItemAvatar>
                <Avatar
                  alt="bootloader icon"
                  src="/img/index/anatomy/bootloader.svg"
                  variant="square"
                />
              </ListItemAvatar>
              <ListItemText
                primary="Bootloader"
                secondary={
                  'Luos Engine allows you to update any firmware of your device, from anywhere.'
                }
              />
            </ListItem>

            <ListItem alignItems="flex-start" sx={{ marginBottom: '50px' }}>
              <ListItemAvatar>
                <Avatar
                  alt="timestamp icon"
                  src="/img/index/anatomy/timestamp.svg"
                  variant="square"
                />
              </ListItemAvatar>
              <ListItemText
                primary="Timestamp"
                secondary={
                  'Luos Engine provides you a distributed timestamp management system.'
                }
              />
            </ListItem>

            <ListItem alignItems="flex-start">
              <ListItemAvatar>
                <Avatar
                  alt="glass icon"
                  src="/img/index/anatomy/glass.svg"
                  variant="square"
                />
              </ListItemAvatar>
              <ListItemText
                primary="Monitoring"
                secondary={
                  'You can control and monitor your device with several SDK (Python, TS, Browser app, and others soon)'
                }
              />
            </ListItem>
          </List>
        </Grid>
      </Grid>
    </div>
  );
};
export default Anatomy;
