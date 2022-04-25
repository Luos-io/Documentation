import React, { useState, useEffect } from 'react';
import styles from './integration.module.css';
import Grid from '@mui/material/Grid';
import Button from '@mui/material/Button';

const Integration = (props) => {
  const [currentImage, setCurrentImage] = useState('mcu');
  const integrations = ['mcu', 'os', 'api', 'sdk'];

  let index = 0;
  useEffect(() => {
    const intervalId = setInterval(() => {
      setCurrentImage(integrations[index]);
      index = ++index % integrations.length;
    }, 3000);

    return () => clearInterval(intervalId);
  }, []);

  return (
    <div className={styles.container}>
      <Grid container spacing={3}>
        <Grid item md={4} lg={4}>
          {' '}
          <img src="img/index/left-lines.svg" className={styles.imgLeft} />
        </Grid>
        <Grid item md={4} lg={4}>
          <h1 className={`${styles.title} ${styles.underline}`}>
            Integrations
          </h1>
        </Grid>
      </Grid>
      <Grid container spacing={3} sx={{ padding: '30px' }}>
        <Grid item md={2} lg={2}></Grid>
        <Grid item md={8} lg={8} style={{ textAlign: 'center' }}>
          <img src={`img/index/integration/${currentImage}.svg`} />
          <div className={styles.btnContainer}>
            <Button variant="contained" className={styles.pinkBtn}>
              Learn more about integrations
            </Button>
          </div>
        </Grid>
        <Grid item md={2} lg={2}></Grid>
      </Grid>
      <Grid container spacing={3}>
        <Grid item md={8} lg={8}></Grid>
        <Grid item md={4} lg={4}>
          <img src="img/index/right-lines.svg" className={styles.imgRight} />
        </Grid>
      </Grid>
    </div>
  );
};
export default Integration;
