import React from 'react';
import styles from './header.module.css';
import Grid from '@mui/material/Grid';
import Button from '@mui/material/Button';
import Writer from '@site/src/components/writer';
import RocketLaunchIcon from '@mui/icons-material/RocketLaunch';
import KeyboardArrowDownIcon from '@mui/icons-material/KeyboardArrowDown';

const Header = (props) => {
  return (
    <div className={styles.container}>
      {' '}
      <Grid container>
        <Grid item md={7} style={{ borderBottom: '15px solid #81c8be' }}>
          <div className={styles.head}>
            {' '}
            <img
              src="img/index/header/buttons.svg"
              style={{ padding: '15px' }}
            />
          </div>
          <Grid container>
            <Grid item md={1} lg={1} className={styles.docs}>
              <img src="img/index/header/docs.svg" />
              <img src="img/index/header/zoom.svg" />
              <img src="img/index/header/luos.svg" />
            </Grid>

            <Grid item md={11} lg={11} pl={3} className={styles.player}>
              <Writer
                textStyle={{
                  color: '#BD99FF',
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
                multiTextDelay={3000}
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
      <Grid container>
        <Grid item md={6} lg={6}></Grid>
        <Grid
          item
          md={6}
          mt={-10}
          style={{
            zIndex: '4',
            borderBottom: '15px solid #81c8be',
            paddingLeft: '0 !important',
          }}
        >
          <div className={styles.head}>
            {' '}
            <img
              src="img/index/header/buttons.svg"
              style={{ padding: '15px' }}
            />
          </div>
          <Grid container>
            <Grid item md={1} lg={1} className={styles.docs}>
              <img src="img/index/header/docs.svg" />
              <img src="img/index/header/zoom.svg" />
              <img src="img/index/header/luos.svg" />
            </Grid>
            <Grid item md={2} lg={2} className={styles.video}>
              <span>
                <KeyboardArrowDownIcon className={styles.cardIcons} /> Video
              </span>
              <div className={styles.engine}>What is Luos Engine?</div>
            </Grid>
            <Grid item md={9} lg={9} className={styles.player}>
              <iframe
                className="player_iframe"
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
