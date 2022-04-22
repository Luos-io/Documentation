import React from 'react';
import styles from './header.module.css';
import Grid from '@mui/material/Grid';
import Button from '@mui/material/Button';
import Writer from '@site/src/components/writer';
import RocketLaunchIcon from '@mui/icons-material/RocketLaunch';

const Header = (props) => {
  return (
    <div className={styles.container}>
      {' '}
      <Grid container>
        <Grid item md={9} className={styles.left}>
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
        <Grid item md={3}></Grid>
      </Grid>
      <Grid container>
        <Grid item md={6}></Grid>
        <Grid item md={6} className={styles.right}></Grid>
      </Grid>
    </div>
  );
};
export default Header;
