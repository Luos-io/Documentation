import React, { useState } from 'react';
import { Paper } from '@mui/material';
import Box from '@mui/material/Box';
import Grid from '@mui/material/Grid';
import Chip from '@mui/material/Chip';
import FormControl from '@mui/material/FormControl';
import Select from '@mui/material/Select';
import InputLabel from '@mui/material/InputLabel';
import MenuItem from '@mui/material/MenuItem';
import ToggleButton from '@mui/material/ToggleButton';
import ToggleButtonGroup from '@mui/material/ToggleButtonGroup';
import CardGrid from './cardGrid';
import styles from './index.module.css';
import data from './data/dataIntro.json';

const Intro = () => {
  const [filters, setfilters] = useState({
    toc: '',
    tags: [],
    hardware: '',
    category: '',
    level: '',
  });

  const handleFilter = (newLevel, filterName) => {
    if (newLevel === filters[filterName]) {
      newLevel = '';
    }
    setfilters({
      ...filters,
      [filterName]: newLevel,
    });
  };

  let countTutos = 0,
    countHours = 0;

  let tagList = [];
  data.tuto.forEach((tuto) => {
    countTutos++;
    countHours += tuto.toc;
    tuto.tags.forEach((tag) => {
      tagList.indexOf(tag) === -1 ? tagList.push(tag) : null;
    });
  });

  return (
    <div>
      <Paper className={styles.introContainer} elevation={1}>
        {/* Title & Text container */}
        <Grid container spacing={2}>
          <Grid item xs={8}>
            <h2>{data.title}</h2>
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
            <FormControl sx={{ m: 1, minWidth: 200 }} size="small">
              <InputLabel id="toc-label" className={styles.select}>
                Time to complete
              </InputLabel>
              <Select
                labelId="toc-label"
                id="toc"
                value={filters.toc}
                label="Time to complete"
                onChange={(e) => {
                  handleFilter(e.target.value, 'toc');
                }}
              >
                <MenuItem value="">All</MenuItem>
                {data.filters.toc.map((filter, index) => (
                  <MenuItem value={filter.duration} key={index}>
                    {filter.label}
                  </MenuItem>
                ))}
              </Select>
            </FormControl>
            <FormControl sx={{ m: 1, minWidth: 120 }} size="small">
              <InputLabel id="topic-label" className={styles.select}>
                Tags
              </InputLabel>
              <Select
                value={filters.tags}
                label="Tags"
                labelId="topic-label"
                onChange={(e) => {
                  handleFilter(e.target.value, 'tags');
                }}
                multiple
                renderValue={(selected) => (
                  <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 0.5 }}>
                    {selected.map((value) => (
                      <Chip key={value} label={value} />
                    ))}
                  </Box>
                )}
              >
                <MenuItem value="">All</MenuItem>
                {tagList.map((label, index) => (
                  <MenuItem value={label} key={index}>
                    {label}
                  </MenuItem>
                ))}
              </Select>
            </FormControl>
            <FormControl sx={{ m: 1, minWidth: 120 }} size="small">
              <InputLabel id="hardware-label" className={styles.select}>
                Hardware
              </InputLabel>
              <Select
                value={filters.hardware}
                label="Hardware"
                labelId="hardware-label"
                onChange={(e) => {
                  handleFilter(e.target.value, 'hardware');
                }}
              >
                <MenuItem value="">All</MenuItem>
                {data.filters.hardware.map((label, index) => (
                  <MenuItem value={label} key={index}>
                    {label}
                  </MenuItem>
                ))}
              </Select>
            </FormControl>
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
                    {label}
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
