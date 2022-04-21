import React from 'react';
import styles from './software.module.css';
import Grid from '@mui/material/Grid';

const Software = (props) => {
  return (
    <div className={styles.container}>
      <h1 className={`${styles.title} ${styles.underline}`}>
        Develop your Edge and Embedded distributed software scalable
      </h1>
      <Grid container spacing={3} mb={5}>
        <Grid item md={1}></Grid>
        <Grid item md={10}>
          <Grid container spacing={3}>
            <Grid item md={3} className={styles.green}>
              <h3 className={`${styles.subtitle} ${styles.underline}`}>
                Network agnostic data format
              </h3>
              <p>
                Text de blablablab lablabla bla bloubloublouh bhbduehbce
                uhecbhu”ecb uhebche”cb”heucbbbbc
              </p>
            </Grid>

            <Grid item md={3} className={styles.purple}>
              <h3 className={`${styles.subtitle} ${styles.underline}`}>
                Portable and scalable code
              </h3>
              <p>
                Text de blablablab lablabla bla bloubloublouh bhbduehbce
                uhecbhu”ecb uhebche”cb”heucbbbbc
              </p>
            </Grid>

            <Grid item md={3} className={styles.orange}>
              <h3 className={`${styles.subtitle} ${styles.underline}`}>
                Encapsulate features into services
              </h3>
              <p>
                Text de blablablab lablabla bla bloubloublouh bhbduehbce
                uhecbhu”ecb uhebche”cb”heucbbbbc
              </p>
            </Grid>

            <Grid item md={3} className={styles.yellow}>
              <h3 className={`${styles.subtitle} ${styles.underline}`}>
                Team work and collaboration
              </h3>
              <p>
                Text de blablablab lablabla bla bloubloublouh bhbduehbce
                uhecbhu”ecb uhebche”cb”heucbbbbc
              </p>
            </Grid>
          </Grid>
        </Grid>
        <Grid item md={1}></Grid>
      </Grid>
      <img src="/img/index/how-its-work.svg" />
    </div>
  );
};
export default Software;
