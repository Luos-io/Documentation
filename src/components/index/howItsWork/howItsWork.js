import React from 'react';
import styles from './howItsWork.module.css';
import Grid from '@mui/material/Grid';

const HowItsWork = (props) => {
  return (
    <div className={styles.container}>
      <h1 className={`${styles.title} ${styles.underline}`}>How its work</h1>

      <Grid container spacing={3}>
        <Grid item md={1}></Grid>
        <Grid item md={10}>
          <Grid container spacing={3}>
            <Grid item md={4} style={{ marginTop: '50px' }}>
              <img src="/img/index/eclipse.svg" className={styles.img} />
              <span className={styles.label}>
                Luos Engine translate your embedded features into APIs
              </span>
            </Grid>
            <Grid item md={4}>
              <img src="/img/index/eclipse.svg" className={styles.img} />
              <span className={styles.label}>
                Luos makes these APIs accessible from anywhere
              </span>
            </Grid>
            <Grid item md={4} style={{ marginTop: '-50px' }}>
              <img src="/img/index/eclipse.svg" className={styles.img} />
              <span className={styles.label}>
                Luos provides you integrations and tools to design your dreamed
                software architectures
              </span>
            </Grid>
          </Grid>
        </Grid>
        <Grid item md={1}></Grid>
      </Grid>
    </div>
  );
};
export default HowItsWork;
