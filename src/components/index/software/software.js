import React from 'react';
import styles from './software.module.css';
import Grid from '@mui/material/Grid';
import Button from '@mui/material/Button';

const Software = (props) => {
  return (
    <div className={styles.container}>
      <h4 style={{ textAlign: 'center' }}>Split the monolith: </h4>
      <h1 className={`${styles.title} ${styles.underline}`}>
        Develop your Edge and Embedded distributed software scalable
      </h1>
      <Grid container spacing={3} mb={5}>
        <Grid item md={1.5} lg={1.5} xl={1.5}></Grid>
        <Grid item md={10} lg={10}>
          <Grid item md={1} lg={1}></Grid>
          <Grid container spacing={3}>
            <Grid item md={3} lg={3} className={styles.green}>
              <h3 className={`${styles.subtitle} ${styles.underline}`}>
                Network agnostic data format
              </h3>
              <p>
                Luos engine provides a network agnostic multimaster data format.
                No matter how specific your network is.
              </p>
            </Grid>
            <Grid item md={1} lg={1}></Grid>
            <Grid item md={3} lg={3} className={styles.purple}>
              <h3 className={`${styles.subtitle} ${styles.underline}`}>
                Portable and scalable code
              </h3>
              <p>
                Luos engine allows you to create packages and make them portable
                and scalable. You can share them with your team or the
                community.
              </p>
            </Grid>
            <Grid item md={1} lg={1}></Grid>
            <Grid item md={3} lg={3} className={styles.orange}>
              <h3 className={`${styles.subtitle} ${styles.underline}`}>
                Encapsulate features into services
              </h3>
              <p>
                Luos engine encapsulates embedded features in services with
                APIs. This allows you to have direct access to the hardware.
              </p>
            </Grid>
          </Grid>
        </Grid>
      </Grid>
      <div className={styles.btnContainer}>
        <Button variant="contained" className={styles.whiteBtn}>
          UNLEASH EMBEDDED SYSTEMS
        </Button>
      </div>
      <img src="/img/index/how-its-work.svg" className={styles.work} />
    </div>
  );
};
export default Software;
