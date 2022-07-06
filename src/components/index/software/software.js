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
    <img src="img/index/software/icons/step1.svg" />,
    <img src="img/index/software/icons/step2.svg" />,
    <img src="img/index/software/icons/step3.svg" />,
  ];

  return (
    <div className={styles.container}>
      <h4 style={{ textAlign: 'center', marginTop: '30px' }}>
        Split the monolith:{' '}
      </h4>
      <h2 className={`${styles.title} ${styles.underline}`}>
        Develop scalable edge and embedded distributed software
      </h2>
      <Grid container spacing={3} mb={5}>
        <Grid item md={1.5} lg={1.5} xl={1.5}></Grid>
        <Grid item md={10} lg={10}>
          <Grid item md={1} lg={1}></Grid>
          <Grid container spacing={3}>
            <Grid item md={3} lg={3} className={styles.green}>
              <h3 className={`${styles.subtitle} ${styles.underline}`}>
                Network agnostic data format
              </h3>
              <p style={{ textAlign: 'justify' }}>
                Luos engine provides a network agnostic multimaster data format.
                No matter how specific your network is.
              </p>
            </Grid>
            <Grid item md={1} lg={1}></Grid>
            <Grid item md={3} lg={3} className={styles.purple}>
              <h3 className={`${styles.subtitle} ${styles.underline}`}>
                Portable and scalable code
              </h3>
              <p style={{ textAlign: 'justify' }}>
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
              <p style={{ textAlign: 'justify' }}>
                Luos engine encapsulates embedded features in services with
                APIs. This allows you to have direct access to the hardware.
              </p>
            </Grid>
          </Grid>
        </Grid>
      </Grid>
      <div className={styles.btnContainer}>
        <Button
          variant="contained"
          className={styles.pinkBtn}
          href="/blog/a-way-to-unleash-embedded-systems"
        >
          Read our blog posts
        </Button>
      </div>
      <BrowserView>
        <Grid container className={styles.mobileNone}>
          <Grid item md={1} lg={1} xl={1}></Grid>
          <Grid item md={10} lg={10} xl={10} style={{ marginBottom: '-50px' }}>
            <div className={styles.head}>
              {' '}
              <Grid container>
                <Grid item md={4} lg={4} xl={4}>
                  {' '}
                  <img
                    src="img/index/header/buttons.svg"
                    style={{ padding: '15px', width: '100px' }}
                  />
                </Grid>
                <Grid item md={4} lg={4} xl={4} style={{ textAlign: 'center' }}>
                  {' '}
                  <h2 className={`${styles.hiwTitle} ${styles.underline}`}>
                    /How it works
                  </h2>
                </Grid>
                <Grid item md={4} lg={4} xl={4}></Grid>
              </Grid>
            </div>
            <Grid container>
              <Grid item md={1} lg={1} xl={1} className={styles.docs}>
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

              <Grid item md={11} lg={11} className={styles.playerLeft}>
                <Grid container>
                  <Grid item md={4} lg={4} xl={4} className={styles.step}>
                    <div className={styles.file}>
                      <div className={styles.fileName}>step_1.c</div>
                    </div>
                    <img
                      src={
                        isDarkTheme
                          ? `img/index/software/step1.svg`
                          : `img/index/software/step1-dark.svg`
                      }
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
                          ? `img/index/software/step2.svg`
                          : `img/index/software/step2-dark.svg`
                      }
                      className={styles.carousselImgDesktop}
                      loading="lazy"
                    />
                    <h3 className={styles.carousselTitle}>2. EXPOSITION</h3>
                    <p className={styles.carousselText}>
                      Luos makes these APIs accessible from anywhere.
                    </p>
                  </Grid>
                  <Grid item md={4} lg={4} xl={4}>
                    <div className={styles.file}>
                      <div className={styles.fileName}>step_3.js</div>
                    </div>
                    <img
                      src={
                        isDarkTheme
                          ? `img/index/software/step3.svg`
                          : `img/index/software/step3-dark.svg`
                      }
                      className={styles.carousselImgDesktop}
                      loading="lazy"
                    />
                    <h3 className={styles.carousselTitle}>3. EXPLOITATION</h3>
                    <p className={styles.carousselText}>
                      Luos provides you with integrations and tools to design
                      the software architectures of your dreams.
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
        </Grid>
      </BrowserView>
      <MobileView>
        <Grid containe className={styles.howitsworkMobile}>
          <Grid item xs={12}>
            <div className={styles.head}>
              <Grid container>
                <Grid item xs={4}>
                  {' '}
                  <img
                    src="img/index/header/buttons.svg"
                    style={{ padding: '15px', width: '100px' }}
                  />
                </Grid>
                <Grid item xs={8} style={{}}>
                  {' '}
                  <h2
                    className={`${styles.hiwTitle} ${styles.underline}`}
                    style={{ marginLeft: '10px' }}
                  >
                    /How its work
                  </h2>
                </Grid>
              </Grid>
            </div>
            <Grid container>
              <Grid item xs={2} className={styles.docs}>
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

              <Grid item xs={10} className={styles.player}>
                <Carousel
                  IndicatorIcon={anArrayOfNumbers}
                  indicatorIconButtonProps={{
                    style: {
                      padding: '10px', // 1
                      filter: 'brightness(50%)',
                    },
                  }}
                  activeIndicatorIconButtonProps={{
                    style: {
                      filter: 'brightness(100%)',
                    },
                  }}
                >
                  {items.map((item, i) => (
                    <Item key={i} item={item} />
                  ))}
                </Carousel>
              </Grid>
            </Grid>
            <Grid item xs={12} className={styles.footer}>
              {' '}
            </Grid>
          </Grid>
        </Grid>
      </MobileView>
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
            ? `img/index/software/${props.item.img}.svg`
            : `img/index/software/${props.item.img}-dark.svg`
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
