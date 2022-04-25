/**
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */
import React from 'react';
import clsx from 'clsx';
import TOCItems from '@theme/TOCItems';
import styles from './styles.module.css';
import { Paper } from '@mui/material';
import Grid from '@mui/material/Grid';
import Avatar from '@mui/material/Avatar';
import Stack from '@mui/material/Stack';
import Requirement from '../../components/school/article/requirement';
import content from '../../components/school/index/data/content';
import data from '../../components/school/index/data/dataIntro';
import authors from '../../components/school/index/data/authors';

const LINK_CLASS_NAME = 'table-of-contents__link toc-highlight';
const LINK_ACTIVE_CLASS_NAME = 'table-of-contents__link--active';

function TOC({ className, ...props }) {
  const link = props.link;
  const regex = /tutorials/g;
  const found = link.match(regex);
  const parentName = props.parentName;
  let author = {};
  let list = [];
  if (found) {
    let currentTutoId = null;
    data['tuto'].forEach((tuto, id) => {
      if (tuto.id === parentName) {
        currentTutoId = id;
      }
    });
    author =
      currentTutoId !== null ? authors[data.tuto[currentTutoId].author] : {};
    if (content[parentName] && content[parentName].content) {
      Object.keys(content[parentName].content).forEach((key) => {
        list.push(content[parentName].content[key]);
      });
    }
  }

  return (
    <div className={clsx(styles.tableOfContents, 'thin-scrollbar', className)}>
      <TOCItems
        {...props}
        linkClassName={LINK_CLASS_NAME}
        linkActiveClassName={LINK_ACTIVE_CLASS_NAME}
      />
      {found && author.name ? (
        <div className={styles.relatedContainer}>
          <Paper elevation={2} className={styles.authorContainer}>
            {/* <h3 className={styles.aboutTheAuthor}>About the Author:</h3> */}
            <Grid container spacing={2}>
              <Grid item md={9}>
                <h3 className={styles.author}>The author: {author.name}</h3>
                <span className={styles.authorDesc}>{author.job}</span>
                <hr className={styles.separator} />
              </Grid>
              <Grid item md={3}>
                <Avatar
                  alt={author.name !== 'nicoR' ? author.name : 'N'}
                  src={`/img/school/authors/${author.img}`}
                  sx={{ width: 56, height: 56, float: 'right' }}
                />
              </Grid>
            </Grid>
            <p className={styles.authorText}>{author.desc}</p>
          </Paper>
          <Stack direction="row" spacing={2} sx={{ justifyContent: 'center' }}>
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
          </Stack>
          <Requirement title="Related Content" color="#FFFFFF" list={list} />
        </div>
      ) : null}
    </div>
  );
}

export default TOC;
