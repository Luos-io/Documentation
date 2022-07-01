import React, { useState, useEffect } from 'react';
import List from '@mui/material/List';
import ListItem from '@mui/material/ListItem';
import ListItemText from '@mui/material/ListItemText';
import Link from '@mui/material/Link';
import Button from '@mui/material/Button';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import ExpandLessIcon from '@mui/icons-material/ExpandLess';
import styles from './article.module.css';

export const Requirement = (props) => {
  const completeList = props.list;
  const shortList = props.list.slice(0, props.shortListSize);
  const [displayList, setDisplayList] = useState(props.shortList ? shortList : completeList);
  const [open, setOpen] = useState(false);

  const handleOpen = () => {
    setDisplayList(open ? shortList : completeList);
    setOpen(!open);
  };

  return (
    <div
      className={styles.requirement}
      style={{
        backgroundColor: props.color,
      }}
    >
      <h3 className={styles.titleRequirement}>{props.title}</h3>
      <hr className={styles.separator} />
      <List>
        {displayList.map((element, index) => (
          <ListItem key={index} component={Link} href={element.link ? element.link : null}>
            <ListItemText
              primary={element.label}
              secondary={element.desc ? element.desc : null}
              className={styles.listItem}
            />
          </ListItem>
        ))}
      </List>
      {props.shortList == true ? (
        <Button onClick={handleOpen} className={styles.showMoreBtn} disableRipple>
          {open ? (
            <ExpandLessIcon className={styles.expandBtn} />
          ) : (
            <ExpandMoreIcon className={styles.expandBtn} />
          )}
        </Button>
      ) : null}
    </div>
  );
};

export default Requirement;
