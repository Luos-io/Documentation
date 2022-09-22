import React, { useState } from 'react';
import { useHistory } from 'react-router-dom';
import { useLocation } from '@docusaurus/router';
import { Paper } from '@mui/material';
import Grid from '@mui/material/Grid';
import FormControl from '@mui/material/FormControl';
import Select from '@mui/material/Select';
import InputLabel from '@mui/material/InputLabel';
import MenuItem from '@mui/material/MenuItem';
import ToggleButton from '@mui/material/ToggleButton';
import ToggleButtonGroup from '@mui/material/ToggleButtonGroup';

import CardGrid from './cardGrid';

import styles from './index.module.css';
import data from './data/dataIntro.json';

const Intro = (props) => {
  const { search } = useLocation();
  const { replace } = useHistory();

  let defaultFilters = {
      toc: '',
      tags: [],
      category: props.category,
      level: '',
    },
    countTutos = 0,
    countHours = 0,
    tagList = [];

  data.tuto.forEach((tuto) => {
    countTutos++;
    countHours += tuto.toc;
    tuto.tags.forEach((tag) => {
      tagList.indexOf(tag) === -1 ? tagList.push(tag) : null;
    });
  });

  new URLSearchParams(search).forEach((value, key) => {
    if (key === 'tags') {
      defaultFilters[key] = value
        .split(',')
        .filter((tag) => tag !== '')
        .map((tag) => {
          const result = tagList.findIndex((t) => t.toLowerCase() === tag);
          return result !== -1 ? tagList[result] : null;
        });
    } else {
      defaultFilters[key] = value;
    }
  });
  const [filters, setfilters] = useState(defaultFilters);

  const handleFilter = (newLevel, filterName) => {
    if (newLevel === filters[filterName]) {
      newLevel = '';
    }

    const newState = {
      ...filters,
      [filterName]: newLevel,
    };
    setfilters(newState);
    const newPath = new URLSearchParams(
      Object.fromEntries(
        Object.entries(newState).filter(
          ([_key, val]) =>
            (!Array.isArray(val) && val !== '') || (Array.isArray(val) && val.length > 0),
        ),
      ),
    );
    replace(`?${newPath.toString().toLowerCase()}`);
  };

  return (
    <div>
      <Paper className={styles.introContainer} elevation={1}>
        {/* Title & Text container */}
        <Grid container spacing={2}>
          <Grid item xs={8}>
            <h1>{props.title}</h1>
          </Grid>
          <Grid item xs={4}>
            {/* TODO set value with JSON */}
            <span className={styles.stats}>
              {countTutos} tutorials | {Math.round(countHours / 60)} Hours
            </span>
          </Grid>
          <Grid item xs={10}>
            <p className={styles.introText}>{data.introText}</p>
          </Grid>
        </Grid>
        {/* Filter container */}
        <Grid container spacing={2}>
          <Grid item xs={12}>
            <FormControl sx={{ m: 1, minWidth: 120 }} size="small">
              <InputLabel id="category-label" className={styles.select}>
                Category
              </InputLabel>
              <Select
                value={filters.category}
                label="Category"
                labelId="category-label"
                onChange={(e) => {
                  handleFilter(e.target.value, 'category');
                }}
              >
                <MenuItem value="">All</MenuItem>
                {data.filters.category.map((label, index) => (
                  <MenuItem value={label} key={index}>
                    {label.charAt(0).toUpperCase() + label.slice(1)}
                  </MenuItem>
                ))}
              </Select>
            </FormControl>
          </Grid>
        </Grid>
        <Grid container spacing={2}>
          <Grid item sx={12}>
            <ToggleButtonGroup
              className={styles.lvlBtn}
              color="success"
              value={filters.level}
              exclusive
              onChange={(e) => {
                handleFilter(e.target.value, 'level');
              }}
              aria-label="text alignment"
            >
              <ToggleButton value="1" aria-label="left aligned">
                Beginner
              </ToggleButton>
              <ToggleButton value="2" aria-label="centered">
                Confirmed
              </ToggleButton>
              <ToggleButton value="3" aria-label="right aligned">
                Expert
              </ToggleButton>
            </ToggleButtonGroup>
          </Grid>
        </Grid>
      </Paper>

      <CardGrid selection={data.tuto} filter={filters} />
    </div>
  );
};

export default Intro;
