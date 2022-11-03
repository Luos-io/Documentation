import React from 'react';
import clsx from 'clsx';
import TOCItems from '@theme/TOCItems';
import styles from './styles.module.css';
import Requirement from '../../components/school/article/requirement';
import content from '../../components/school/index/data/content';
import data from '../../components/school/index/data/dataIntro';

const LINK_CLASS_NAME = 'table-of-contents__link toc-highlight';
const LINK_ACTIVE_CLASS_NAME = 'table-of-contents__link--active';

function TOC({ className, ...props }) {
  const link = props.link;
  const regex = /tutorials/g;
  const found = link.match(regex);
  const parentName = props.parentName;
  let author = {};
  let list = [];
  let hardware = [];
  if (found) {
    let currentTutoId = null;
    data['tuto'].forEach((tuto, id) => {
      if (tuto.id === parentName) {
        currentTutoId = id;
      }
    });
    if (content[parentName] && content[parentName].content) {
      Object.keys(content[parentName].content).forEach((key) => {
        list.push(content[parentName].content[key]);
      });
    }
    if (content[parentName] && content[parentName].hardware) {
      Object.keys(content[parentName].hardware).forEach((key) => {
        hardware.push(content[parentName].hardware[key]);
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
      {found ? (
        <div className={styles.relatedContainer}>
          {hardware.length > 0 ? (
            <Requirement
              title="Supported hardware"
              color="#FFFFFF"
              list={hardware}
              shortList={hardware.length > 2 ? true : false}
              shortListSize="2"
            />
          ) : null}

          {list.length > 0 ? (
            <Requirement title="Related content" color="#FFFFFF" list={list} />
          ) : null}
        </div>
      ) : null}
    </div>
  );
}

export default TOC;
