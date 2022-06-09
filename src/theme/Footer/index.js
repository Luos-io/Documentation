/**
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */
import React, { useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import { useThemeConfig } from '@docusaurus/theme-common';
import styles from './styles.module.css';
import GitHubButton from 'react-github-btn';
import Grid from '@mui/material/Grid';
import { useColorMode } from '@docusaurus/theme-common';
import recentPosts from '../../../.docusaurus/docusaurus-plugin-content-blog/default/blog-post-list-prop-default.json';
import dataTuto from '../../components/school/index/data/dataIntro.json';

function Footer() {
  const { isDarkTheme } = useColorMode();
  const { footer } = useThemeConfig();
  const { copyright, links = [], logo = {} } = footer || {};

  if (!footer) {
    return null;
  }

  const lastBlogPosts = recentPosts.items.slice(0, 2);
  const lastTuto = dataTuto.tuto.slice(-2);

  const [lastCommits, setLastCommits] = useState([]);

  fetch('https://api.github.com/repos/Luos-io/luos_engine/commits')
    .then((res) => res.json())
    .then((result) => {
      setLastCommits(result.slice(0, 3));
    });

  return (
    <footer className={clsx('footer')} style={{ padding: '50px' }}>
      <Grid container style={{ marginBottom: '30px' }}>
        <Grid item xs={12} md={4} lg={4} xl={4}>
          {' '}
          <img
            src={isDarkTheme ? 'img/index/powered/luos.svg' : 'img/index/powered/luos-dark.svg'}
            style={{ verticalAlign: 'middle', width: '150px' }}
          />
        </Grid>
        <Grid item xs={12} md={4} lg={4} xl={4}>
          <h1 className={styles.title}>Support us</h1>
          <div className="footer__bottom btn_footer">
            <GitHubButton
              href="https://github.com/luos-io"
              aria-label="Follow @luos-io on GitHub"
              data-size="large"
            >
              Follow @luos-io
            </GitHubButton>

            <GitHubButton
              href="https://github.com/luos-io/Luos"
              data-icon="octicon-star"
              aria-label="Star luos-io/documentation on GitHub"
              data-size="large"
              data-show-count="true"
            >
              Star
            </GitHubButton>
          </div>
        </Grid>
        <Grid item xs={12} md={4} lg={4} xl={4}>
          <h1 className={styles.titleCommunity}>Join our community</h1>
          <div className={styles.joinUsContainer}>
            <a href="https://discord.gg/luos">
              <img src="/img/discord.png" className="rsLogo"></img>
            </a>
            <a href="https://www.reddit.com/r/Luos/">
              <img src="/img/reddit.png" className="rsLogo"></img>
            </a>
            <a href="https://twitter.com/Luos_io">
              <img src="/img/twitter.png" className="rsLogo"></img>
            </a>
            <a href="https://www.linkedin.com/company/luos">
              <img src="/img/linkedin.png" className="rsLogo"></img>
            </a>
          </div>
        </Grid>
      </Grid>

      <Grid container style={{ marginBottom: '30px' }}>
        <Grid item xs={12} md={4} lg={4} xl={4}>
          <h2 className={styles.subtitle}>Latest articles on our blog</h2>
          {lastBlogPosts.map((element, index) => (
            <Link key={index} href={element.permalink} className={styles.link}>
              {element.title}
            </Link>
          ))}
          <Link className={`${styles.link} ${styles.blog}`} to="/blog">
            Our Blog
          </Link>
          <h2 className={styles.subtitle}>Latest tutorials on our documentation</h2>
          {lastTuto.map((element, index) => (
            <Link key={index} href={element.link} className={styles.link}>
              {element.title}
            </Link>
          ))}
          <Link className={`${styles.link} ${styles.tuto}`} to="/tutorials">
            Our Tutorials
          </Link>
        </Grid>
        <Grid item xs={12} md={4} lg={4} xl={4} pr={4}>
          <h2 className={styles.subtitle}>Latest commits on Luos repository</h2>
          {lastCommits.map((element, index) => (
            <Link key={index} href={element.commit.url} className={styles.link}>
              {element.commit.message}
            </Link>
          ))}
        </Grid>
        <Grid item xs={12} md={4} lg={4} xl={4}>
          <h2 className={styles.subtitle}>Learn more about us</h2>
          <Link
            className={`${styles.link} ${styles.blog} ${styles.mbTen}`}
            to="/docs/luos-technology"
          >
            Technology
          </Link>
          <Link
            className={`${styles.link} ${styles.tuto} ${styles.mbTen}`}
            href="https://app.luos.io/"
          >
            Tools
          </Link>
          <span className={` ${styles.span} ${styles.ressources}`} to="/docs/luos-technology">
            Ressources
          </span>
          <ul className={styles.list}>
            <li>
              {' '}
              <Link
                className={`${styles.link} ${styles.ressourcesLink} ${styles.mbZero}`}
                to="/tutorials/get-started"
              >
                Get Started
              </Link>
            </li>
            <li>
              {' '}
              <Link
                className={`${styles.link} ${styles.ressourcesLink} ${styles.mbZero}`}
                to="/tutorials"
              >
                Tutorials
              </Link>
            </li>
            <li>
              {' '}
              <Link
                className={`${styles.link} ${styles.ressourcesLink} ${styles.mbZero}`}
                to="/docs/luos-technology"
              >
                Documentation
              </Link>
            </li>
            <li>
              {' '}
              <Link
                className={`${styles.link} ${styles.ressourcesLink} ${styles.mbZero}`}
                to="/faq"
              >
                Troubleshooting
              </Link>
            </li>
            <li>
              {' '}
              <Link
                className={`${styles.link} ${styles.ressourcesLink} ${styles.mbZero}`}
                to="/blog"
              >
                Blog
              </Link>
            </li>
          </ul>
          <span className={` ${styles.span} ${styles.community}`} to="/docs/luos-technology">
            Community
          </span>
          <ul className={styles.list}>
            <li>
              {' '}
              <Link
                className={`${styles.link} ${styles.communityLink} ${styles.mbZero}`}
                href="https://discord.gg/luos"
              >
                Discord
              </Link>
            </li>
            <li>
              {' '}
              <Link
                className={`${styles.link} ${styles.communityLink} ${styles.mbZero}`}
                href="https://www.reddit.com/r/Luos/"
              >
                Reddit
              </Link>
            </li>
            <li>
              {' '}
              <Link
                className={`${styles.link} ${styles.communityLink} ${styles.mbZero}`}
                href="https://github.com/Luos-io"
              >
                Github
              </Link>
            </li>
          </ul>
        </Grid>
      </Grid>
      {copyright ? (
        <div
          className={styles.copyright}
          // eslint-disable-next-line react/no-danger
          dangerouslySetInnerHTML={{
            __html: copyright,
          }}
        />
      ) : null}
    </footer>
  );
}

export default Footer;
