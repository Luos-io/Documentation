import React from 'react';
import styles from './powered.module.css';
import Grid from '@mui/material/Grid';
import List from '@mui/material/List';
import ListItem from '@mui/material/ListItem';
import ListItemText from '@mui/material/ListItemText';
import ListItemAvatar from '@mui/material/ListItemAvatar';
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
import Link from '@mui/material/Link';

const Powered = (props) => {
  const { isDarkTheme } = useColorMode();
  return (
    <div className={styles.container}>
      <div className={styles.containerTitle}>
        {' '}
        <h2 className={`${styles.title}`}>
          More features powered by{' '}
          <img
            src={isDarkTheme ? 'img/index/powered/luos.svg' : 'img/index/powered/luos-dark.svg'}
            style={{ verticalAlign: 'middle', width: '100px' }}
          />
          <span style={{ display: 'none' }}>Luos</span>
        </h2>
      </div>
      <Grid container mt={10} mb={10}>
        <Grid item md={2} lg={2} xl={2}></Grid>
        <Grid item xs={12} md={4} lg={4} xl={4}>
          <List sx={{ width: '100%' }}>
            <ListItem alignitems="flex-start">
              <ListItemAvatar>
                <PeopleAltIcon fontSize="large" className={styles.listIcon} />
              </ListItemAvatar>
              <ListItemText
                className={styles.text}
                primary="Multimaster"
                secondary={
                  'Any service can control any other, which allows you to have a fully distributed approach, or not...'
                }
              />
            </ListItem>

            <ListItem
              alignitems="flex-start"
              component={Link}
              href="/docs/luos-technology/services#service-properties"
              className={styles.listLink}
            >
              <ListItemAvatar>
                <AbcIcon fontSize="large" className={styles.listIcon} />
              </ListItemAvatar>
              <ListItemText
                className={`${styles.text} ${styles.textLink}`}
                primary="Service aliases management"
                secondary={
                  'Luos provides you with a way to automatically retrieve values, so you don’t need to poll them.'
                }
              />
            </ListItem>
            <ListItem alignitems="flex-start">
              <ListItemAvatar>
                <CachedIcon fontSize="large" className={styles.listIcon} />
              </ListItemAvatar>
              <ListItemText
                className={styles.text}
                primary="Data auto-update"
                secondary={
                  'Each service can have a convenient alias, allowing you to find and use it easily.'
                }
              />
            </ListItem>
          </List>
        </Grid>
        <Grid item xs={12} md={4} lg={4} xl={4}>
          <List sx={{ width: '100%' }} className={styles.secondList}>
            <ListItem alignitems="flex-start">
              <ListItemAvatar>
                <EventNoteIcon fontSize="large" className={styles.listIcon} />
              </ListItemAvatar>
              <ListItemText
                className={styles.text}
                primary="Event-based"
                secondary={
                  'You can choose to poll or wait for a callback for each event received by your service.'
                }
              />
            </ListItem>

            <ListItem
              alignitems="flex-start"
              component={Link}
              href="/docs/luos-technology/message/object-dictionary"
              className={styles.listLink}
            >
              <ListItemAvatar>
                <MenuBookIcon fontSize="large" className={styles.listIcon} />
              </ListItemAvatar>
              <ListItemText
                className={styles.text}
                primary="Object dictionnary"
                secondary={
                  'Luos is able to convert any unit on the fly, so there are no problems with data units.'
                }
              />
            </ListItem>

            <ListItem
              alignitems="flex-start"
              component={Link}
              href="/docs/luos-technology/services/routing-table#detection"
              className={styles.listLink}
            >
              <ListItemAvatar>
                <ExploreIcon fontSize="large" className={styles.listIcon} />
              </ListItemAvatar>
              <ListItemText
                className={styles.text}
                primary="Detection, hotplug & selfhealing"
                secondary={
                  'Luos engine manages all your services at every step: creation, detection, hotplug, usage, sanity check, and exclusion in case of problems.'
                }
              />
            </ListItem>
          </List>
        </Grid>
        <Grid item md={2} lg={2} xl={2}></Grid>
      </Grid>
      <Grid container>
        <Grid item xs={12} md={6} lg={6} className={styles.nebula}>
          <h2 className={`${styles.titleFooter} ${styles.underline}`}>
            Develop and scale your edge and embedded distributed software now
          </h2>
          <p className={styles.textGs}>
            Get started with Luos by setting up your development environment with your IDE and build
            your first embedded microservice.
          </p>
          <div className={styles.btnContainer}>
            <Button variant="contained" className={styles.pinkBtn} href="/tutorials/get-started">
              Get started <RocketLaunchIcon style={{ marginLeft: '10px' }} />
            </Button>
            <Button
              variant="contained"
              className={styles.whiteBtn}
              href="https://discord.gg/luos"
              rel="nofollow"
            >
              Join the community
            </Button>
          </div>
        </Grid>
        <Grid item md={6} lg={6} style={{ marginTop: '-50px' }} className={styles.mobileNone}>
          <div className={styles.head}>
            {' '}
            <img src="img/index/header/buttons.svg" style={{ padding: '15px', width: '100px' }} />
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
