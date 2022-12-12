import React from 'react';
import { BrowserView, MobileView } from 'react-device-detect';
import styles from './software.module.css';
import Grid from '@mui/material/Grid';
import Button from '@mui/material/Button';
import { useColorMode } from '@docusaurus/theme-common';
import Carousel from 'react-material-ui-carousel';
import Paper from '@mui/material/Paper';
import FileCopyIcon from '@mui/icons-material/FileCopy';
import SearchIcon from '@mui/icons-material/Search';

const Software = (props) => {
  const { isDarkTheme } = useColorMode();

  let items = [
    {
      name: '1. ENCAPSULATION',
      description: 'Luos engine translates your embedded features into APIs.',
      file: 'STEP_1.c',
      img: 'step1',
    },
    {
      name: '2. EXPOSITION',
      description: 'Luos makes these APIs accessible from anywhere.',
      file: 'STEP_2.py',
      img: 'step2',
    },
    {
      name: '3. EXPLOITATION',
      description:
        'Luos provides you with integrations and tools to design the software architectures of your dreams.',
      file: 'STEP_3.js',
      img: 'step3',
    },
  ];

  const anArrayOfNumbers = [
    <img src="assets/images/index/software/icons/step1.svg" />,
    <img src="assets/images/index/software/icons/step2.svg" />,
    <img src="assets/images/index/software/icons/step3.svg" />,
  ];

  return (
    <div className={styles.container}>
      <h2 className={`${styles.title} ${styles.underline}`}>
        Develop scalable cyber-physical systems
      </h2>
      <Grid container>
        <Grid item md={1} lg={1} xl={1}></Grid>
        <Grid item md={10} lg={10} xl={10} style={{ marginBottom: '-50px' }}>
          <div className={styles.head}>
            {' '}
            <Grid container>
              <Grid item xs={'auto'}>
                {' '}
                <img
                  src="assets/images/index/header/buttons.svg"
                  style={{ padding: '15px', width: '100px' }}
                  alt="buttons"
                />
              </Grid>
              <Grid item xs={true} style={{ textAlign: 'center' }}>
                {' '}
                <h2 className={`${styles.hiwTitle} ${styles.underline}`}>/How Luos engine works</h2>
              </Grid>
            </Grid>
          </div>
          <Grid container>
            <Grid item md={1} lg={1} xl={1} className={styles.docs}>
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

            <Grid item md={11} lg={11} className={styles.playerLeft}>
              <Grid container>
                <Grid item md={4} lg={4} xl={4} className={styles.step}>
                  <div className={styles.file}>
                    <div className={styles.fileName}>step_1.c</div>
                  </div>
                  <img
                    src={
                      isDarkTheme
                        ? `assets/images/index/software/step1.svg`
                        : `assets/images/index/software/step1-dark.svg`
                    }
                    alt="step one"
                    className={styles.carousselImgDesktop}
                    loading="lazy"
                  />
                  <h3 className={styles.carousselTitle}>1. ENCAPSULATION</h3>
                  <p className={styles.carousselText}>
                    Luos engine translates your embedded features into APIs.
                  </p>
                </Grid>
                <Grid item md={4} lg={4} xl={4} className={styles.step}>
                  <div className={styles.file}>
                    <div className={styles.fileName}>step_2.py</div>
                  </div>
                  <img
                    src={
                      isDarkTheme
                        ? `assets/images/index/software/step2.svg`
                        : `assets/images/index/software/step2-dark.svg`
                    }
                    alt="step two"
                    className={styles.carousselImgDesktop}
                    loading="lazy"
                  />
                  <h3 className={styles.carousselTitle}>2. EXPOSITION</h3>
                  <p className={styles.carousselText}>
                    Detect and use any service API accessible from anywhere, no matter how specific
                    your network is.
                  </p>
                </Grid>
                <Grid item md={4} lg={4} xl={4}>
                  <div className={styles.file}>
                    <div className={styles.fileName}>step_3.js</div>
                  </div>
                  <img
                    src={
                      isDarkTheme
                        ? `assets/images/index/software/step3.svg`
                        : `assets/images/index/software/step3-dark.svg`
                    }
                    alt="step three"
                    className={styles.carousselImgDesktop}
                    loading="lazy"
                  />
                  <h3 className={styles.carousselTitle}>3. EXPLOITATION</h3>
                  <p className={styles.carousselText}>
                    Compose your product step by step and make it evolutive and future-proof easily.
                  </p>
                </Grid>
              </Grid>
            </Grid>
            <Grid item md={12} lg={12} xl={12} className={styles.footer}>
              {' '}
            </Grid>
          </Grid>
        </Grid>
        <Grid item md={1} lg={1} xl={1}></Grid>
        <Grid item xs={12}>
          <div className={styles.btnContainer}>
            <Button
              variant="contained"
              className={styles.pinkBtn}
              href="/blog/a-way-to-unleash-embedded-systems"
            >
              Read our blog posts
            </Button>
          </div>
        </Grid>
      </Grid>
    </div>
  );
};

function Item(props) {
  const { isDarkTheme } = useColorMode();
  return (
    <Paper className={styles.caroussel}>
      <div className={styles.file}>
        <div className={styles.fileName}>{props.item.file}</div>
      </div>
      <img
        src={
          isDarkTheme
            ? `assets/images/index/software/${props.item.img}.svg`
            : `assets/images/index/software/${props.item.img}-dark.svg`
        }
        className={styles.carousselImg}
        loading="lazy"
      />
      <h3 className={styles.carousselTitle}>{props.item.name}</h3>
      <p className={styles.carousselText}>{props.item.description}</p>
    </Paper>
  );
}

export default Software;
