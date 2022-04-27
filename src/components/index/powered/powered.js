import React from 'react';
import styles from './powered.module.css';
import Grid from '@mui/material/Grid';
import List from '@mui/material/List';
import ListItem from '@mui/material/ListItem';
import ListItemText from '@mui/material/ListItemText';
import ListItemAvatar from '@mui/material/ListItemAvatar';
import Avatar from '@mui/material/Avatar';
import Button from '@mui/material/Button';
import RocketLaunchIcon from '@mui/icons-material/RocketLaunch';

const Powered = (props) => {
  return (
    <div className={styles.container}>
      <div className={styles.containerTitle}>
        {' '}
        <h1 className={`${styles.title} ${styles.underline}`}>
          Powered by Luos
        </h1>
      </div>
      <Grid container mt={3} mb={5}>
        <Grid item md={6} lg={6} pl={7} pr={3}>
          <List sx={{ width: '100%' }}>
            <ListItem alignItems="flex-start">
              <ListItemAvatar>
                <Avatar
                  alt="table tree icon"
                  src="/img/index/powered/detection.svg"
                  variant="square"
                />
              </ListItemAvatar>
              <ListItemText
                className={styles.text}
                primary="Detection, hotplug & selfhealing"
                secondary={
                  'Luos Engine manages all your services during every steps: creation, detection, hotplug, usage, sanity check, and exclusion in case of issues.'
                }
              />
            </ListItem>

            <ListItem alignItems="flex-start">
              <ListItemAvatar>
                <Avatar
                  alt="cubes icon"
                  src="/img/index/powered/data.svg"
                  variant="square"
                />
              </ListItemAvatar>
              <ListItemText
                className={styles.text}
                primary="Data auto-update"
                secondary={
                  'Luos provides you a way to get back automatically values, you don’t have to poll them anymore.'
                }
              />
            </ListItem>

            <ListItem alignItems="flex-start">
              <ListItemAvatar>
                <Avatar
                  alt="files icon"
                  src="/img/index/powered/service.svg"
                  variant="square"
                />
              </ListItemAvatar>
              <ListItemText
                className={styles.text}
                primary="Service aliases management"
                secondary={
                  'Every services can have a convenient alias allowing you to easily find and use them.'
                }
              />
            </ListItem>
          </List>
        </Grid>
        <Grid item md={6} lg={6}>
          <List sx={{ width: '100%' }}>
            <ListItem alignItems="flex-start">
              <ListItemAvatar>
                <Avatar
                  alt="bootloader icon"
                  src="/img/index/powered/object.svg"
                  variant="square"
                />
              </ListItemAvatar>
              <ListItemText
                className={styles.text}
                primary="Object dictionnary"
                secondary={
                  'Luos is able to convert any unit on the fly, to avoid any trouble regarding data units.'
                }
              />
            </ListItem>

            <ListItem alignItems="flex-start">
              <ListItemAvatar>
                <Avatar
                  alt="timestamp icon"
                  src="/img/index/powered/event.svg"
                  variant="square"
                />
              </ListItemAvatar>
              <ListItemText
                className={styles.text}
                primary="Event-based"
                secondary={
                  'You can choose to poll or to wait for a callback for each event received by you service.'
                }
              />
            </ListItem>

            <ListItem alignItems="flex-start">
              <ListItemAvatar>
                <Avatar
                  alt="glass icon"
                  src="/img/index/powered/multimaster.svg"
                  variant="square"
                />
              </ListItemAvatar>
              <ListItemText
                className={styles.text}
                primary="Multimaster"
                secondary={
                  'Any service can control any other allowing you to have a complete distributed approach, or not...'
                }
              />
            </ListItem>
          </List>
        </Grid>
      </Grid>
      <Grid container>
        <Grid item md={6} lg={6} className={styles.nebula}>
          <h1 className={`${styles.title} ${styles.underline}`}>
            Develop and scale your edge and embedded distributed software now
          </h1>
          <p className={styles.textGs}>
            Get started with Luos by setting up your development environment
            with your IDE and build your first embedded microservice.
          </p>
          <div className={styles.btnContainer}>
            <Button variant="contained" className={styles.pinkBtn}>
              Get started <RocketLaunchIcon />
            </Button>
            <Button variant="contained" className={styles.whiteBtn}>
              Join the community
            </Button>
          </div>
        </Grid>
        <Grid item md={6} lg={6} style={{ marginTop: '-50px' }}>
          <div className={styles.head}>
            {' '}
            <img
              src="img/index/header/buttons.svg"
              style={{ padding: '15px' }}
            />
          </div>
          <Grid container style={{ height: '100%' }}>
            <Grid item md={1} lg={1} className={styles.docs}>
              <img src="img/index/header/docs.svg" />
              <img src="img/index/header/zoom.svg" />
              <img src="img/index/header/luos.svg" />
            </Grid>
            <Grid item md={11} lg={11} className={styles.player}></Grid>
          </Grid>
        </Grid>
      </Grid>
    </div>
  );
};
export default Powered;
