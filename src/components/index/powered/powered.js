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
import { useColorMode } from '@docusaurus/theme-common';
import FileCopyIcon from '@mui/icons-material/FileCopy';
import SearchIcon from '@mui/icons-material/Search';
import ExploreIcon from '@mui/icons-material/Explore';
import CachedIcon from '@mui/icons-material/Cached';
import AbcIcon from '@mui/icons-material/Abc';
import MenuBookIcon from '@mui/icons-material/MenuBook';
import EventNoteIcon from '@mui/icons-material/EventNote';
import PeopleAltIcon from '@mui/icons-material/PeopleAlt';

const Powered = (props) => {
  const { isDarkTheme } = useColorMode();
  return (
    <div className={styles.container}>
      <div className={styles.containerTitle}>
        {' '}
        <h1 className={`${styles.title}`}>
          Powered by{' '}
          <img
            src={
              isDarkTheme
                ? 'img/index/powered/luos.svg'
                : 'img/index/powered/luos-dark.svg'
            }
            style={{ verticalAlign: 'middle', width: '100px' }}
          />
        </h1>
      </div>
      <Grid container mt={10} mb={10}>
        <Grid item md={2} lg={2} xl={2}></Grid>
        <Grid item xs={12} md={4} lg={4} xl={4}>
          <List sx={{ width: '100%' }}>
            <ListItem alignItems="flex-start">
              <ListItemAvatar>
                <PeopleAltIcon fontSize="large" className={styles.listIcon} />
              </ListItemAvatar>
              <ListItemText
                className={styles.text}
                primary="Multimaster"
                secondary={
                  'Any service can control any other allowing you to have a complete distributed approach, or not...'
                }
              />
            </ListItem>

            <ListItem alignItems="flex-start">
              <ListItemAvatar>
                <CachedIcon fontSize="large" className={styles.listIcon} />
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
                <AbcIcon fontSize="large" className={styles.listIcon} />
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
        <Grid item xs={12} md={4} lg={4} xl={4}>
          <List sx={{ width: '100%' }}>
            <ListItem alignItems="flex-start">
              <ListItemAvatar>
                <EventNoteIcon fontSize="large" className={styles.listIcon} />
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
                <MenuBookIcon fontSize="large" className={styles.listIcon} />
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
                <ExploreIcon fontSize="large" className={styles.listIcon} />
              </ListItemAvatar>
              <ListItemText
                className={styles.text}
                primary="Detection, hotplug & selfhealing"
                secondary={
                  'Luos Engine manages all your services during every steps: creation, detection, hotplug, usage, sanity check, and exclusion in case of issues.'
                }
              />
            </ListItem>
          </List>
        </Grid>
        <Grid item md={2} lg={2} xl={2}></Grid>
      </Grid>
      <Grid container>
        <Grid item xs={12} md={6} lg={6} className={styles.nebula}>
          <h1 className={`${styles.titleFooter} ${styles.underline}`}>
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
        <Grid
          item
          md={6}
          lg={6}
          style={{ marginTop: '-50px' }}
          className={styles.mobileNone}
        >
          <div className={styles.head}>
            {' '}
            <img
              src="img/index/header/buttons.svg"
              style={{ padding: '15px', width: '100px' }}
            />
          </div>
          <Grid container style={{ height: '100%' }}>
            <Grid item md={1} lg={1} className={styles.docs}>
              <FileCopyIcon fontSize="large" className={styles.icons} />
              <SearchIcon fontSize="large" className={styles.icons} />
              {isDarkTheme ? (
                <>
                  {' '}
                  <img src="img/index/header/luos.svg" />{' '}
                </>
              ) : (
                <>
                  {' '}
                  <img src="img/index/header/luos-white.svg" />{' '}
                </>
              )}
            </Grid>
            <Grid item md={11} lg={11} className={styles.player}></Grid>
          </Grid>
        </Grid>
      </Grid>
    </div>
  );
};
export default Powered;
