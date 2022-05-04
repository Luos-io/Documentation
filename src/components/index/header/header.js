import React from 'react';
import styles from './header.module.css';
import Grid from '@mui/material/Grid';
import Button from '@mui/material/Button';
import Writer from '@site/src/components/writer';
import RocketLaunchIcon from '@mui/icons-material/RocketLaunch';
import KeyboardArrowDownIcon from '@mui/icons-material/KeyboardArrowDown';
import { useColorMode } from '@docusaurus/theme-common';
import FileCopyIcon from '@mui/icons-material/FileCopy';
import SearchIcon from '@mui/icons-material/Search';

const Header = (props) => {
  const { isDarkTheme } = useColorMode();
  return (
    <div className={styles.container}>
      {' '}
      <Grid container>
        <Grid item md={7} style={{ borderBottom: '15px solid #81c8be' }}>
          <div className={styles.head}>
            {' '}
            <img
              src="img/index/header/buttons.svg"
              style={{ padding: '15px', width: '100px' }}
            />
          </div>
          <Grid container>
            <Grid item xs={2} md={1} lg={1} className={styles.docs}>
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

            <Grid
              item
              xs={10}
              md={11}
              lg={11}
              pl={3}
              className={styles.playerLeft}
            >
              <Writer
                textStyle={{
                  color: '#BD99FF',
                  height: '70px',
                }}
                startDelay={0}
                cursorColor="#BD99FF"
                multiTextLoop={true}
                multiText={[
                  'an open-source project_',
                  'an embedded microservice orchestrator_',
                  'an embedded library_',
                  'a distributed IPC_',
                  'a message broker_',
                  'an ecosystem_',
                  'a SDK_',
                  'a project toolset_',
                  'a community_',
                ]}
                multiTextDelay={2000}
                typeSpeed={100}
              />{' '}
              <p className={styles.text}>
                Luos makes it easy to develop and scale your edge and embedded
                distributed software.
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
          </Grid>
        </Grid>

        <Grid item md={5} lg={5}></Grid>
      </Grid>
      <Grid container className={styles.mobileNone}>
        <Grid item md={6} lg={6}></Grid>
        <Grid
          item
          md={6}
          mt={-13}
          style={{
            zIndex: '3',
            borderBottom: '15px solid #81c8be',
            paddingLeft: '0 !important',
          }}
        >
          <div className={styles.head}>
            {' '}
            <img
              src="img/index/header/buttons.svg"
              style={{ padding: '15px', width: '100px' }}
            />
          </div>
          <Grid container>
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
            <Grid item md={2} lg={2} className={styles.video}>
              <span>
                <KeyboardArrowDownIcon className={styles.cardIcons} /> Video
              </span>
              <div className={styles.engine}>What is Luos Engine?</div>
            </Grid>
            <Grid item md={9} lg={9} className={styles.player}>
              <iframe
                className={styles.player_iframe}
                src="https://www.youtube.com/embed/ujh0xNE3TZ8?feature=oembed"
                allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture; fullscreen"
              ></iframe>
            </Grid>
          </Grid>
        </Grid>
      </Grid>
    </div>
  );
};
export default Header;
