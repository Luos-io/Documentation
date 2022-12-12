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
      <Grid container>
        <Grid item xs={12} md={6} lg={6} className={styles.nebula}>
          <h2 className={`${styles.titleFooter} ${styles.underline}`}>
            Develop and scale cyber-physical systems now
          </h2>
          <p className={styles.textGs}>
            Get started with Luos by setting up your development environment and build your first
            embedded microservices ready to be controlled by your digital twin.
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
            <img
              src="assets/images/index/header/buttons.svg"
              style={{ padding: '15px', width: '100px' }}
              alt="buttons"
            />
          </div>
          <Grid container style={{ height: '100%' }}>
            <Grid item md={1} lg={1} className={styles.docs}>
              <FileCopyIcon fontSize="large" className={styles.icons} />
              <SearchIcon fontSize="large" className={styles.icons} />
              {isDarkTheme ? (
                <>
                  {' '}
                  <img src="assets/images/index/header/luos.svg" alt="luos" />{' '}
                </>
              ) : (
                <>
                  {' '}
                  <img src="assets/images/index/header/luos-white.svg" alt="luos-white" />{' '}
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
