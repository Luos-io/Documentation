import React, { useEffect, useState, useRef } from 'react';
import useIsBrowser from '@docusaurus/useIsBrowser';
import { Paper } from '@mui/material';
import Grid from '@mui/material/Grid';
import FormControl from '@mui/material/FormControl';
import Select, { SelectChangeEvent } from '@mui/material/Select';
import InputLabel from '@mui/material/InputLabel';
import MenuItem from '@mui/material/MenuItem';
import Button from '@mui/material/Button';
import ButtonGroup from '@mui/material/ButtonGroup';
import ToggleButton from '@mui/material/ToggleButton';
import ToggleButtonGroup from '@mui/material/ToggleButtonGroup';
//import useIsBrowser from '@docusaurus/useIsBrowser';
import CardGrid from './cardGrid';
import styles from './index.module.css';
import data from './data/dataIntro.json';

export default function Intro() {
  const [selection, setSelection] = useState();
  const [filters, setfilters] = useState({
    toc: '',
    tags: '',
    hardware: '',
    category: '',
    lvl: '',
  });

  const handleToc = (event, newLevel) => {
    const oldFilter = filters;
    setfilters({
      toc: newLevel.props.value,
      tags: oldFilter.tags,
      hardware: oldFilter.hardware,
      category: oldFilter.category,
      lvl: oldFilter.lvl,
    });
    //  setSelection(filters);
  };

  useEffect(() => {
    console.log(filters);
    let filtered = [];

    data.tuto.forEach((tuto) => {
      console.log(tuto);
      if (tuto.tags.includes(filters.tags)) {
        filtered.push(tuto);
      }
    });
    console.log(filtered);
    setSelection(filtered);
    console.log(selection);
  }, [filters]);

  const handleTopic = (event, newLevel) => {
    const oldFilter = filters;
    console.log('old filter');
    console.log(oldFilter);
    setfilters({
      toc: oldFilter.toc,
      tags: newLevel.props.value,
      hardware: oldFilter.hardware,
      category: oldFilter.category,
      lvl: oldFilter.lvl,
    });
    // setSelection(filters);
    console.log('new filter');
    console.log(filters);
  };

  const handleHardware = (event, newLevel) => {
    const oldFilter = filters;
    setfilters({
      toc: oldFilter.toc,
      tags: oldFilter.tags,
      hardware: newLevel.props.value,
      category: oldFilter.category,
      lvl: oldFilter.lvl,
    });
    //  setSelection(filters);
  };

  const handleCategory = (event, newLevel) => {
    const oldFilter = filters;
    setfilters({
      toc: oldFilter.toc,
      tags: oldFilter.tags,
      hardware: oldFilter.hardware,
      category: newLevel.props.value,
      lvl: oldFilter.lvl,
    });
    //  setSelection(filters);
  };

  const handleLevel = (event, newLevel) => {
    const oldFilter = filters;
    setfilters({
      toc: oldFilter.toc,
      tags: oldFilter.tags,
      hardware: oldFilter.hardware,
      category: oldFilter.category,
      lvl: newLevel,
    });
    //  setSelection(filters);
  };

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
            <span className={styles.stats}>X tutorials | X Hours</span>
          </Grid>
          <Grid item xs={10}>
            <p className={styles.introText}>{data.introText}</p>
          </Grid>
        </Grid>
        {/* Filter container */}
        <Grid container spacing={2}>
          <Grid item xs={9}>
            <FormControl sx={{ m: 1, minWidth: 100 }}>
              <InputLabel id="toc-label">Time to complete</InputLabel>
              <Select
                labelId="toc-label"
                id="toc"
                value={filters.toc}
                className={styles.filterBtn}
                sx={{ width: '200px' }}
                onChange={handleToc}
              >
                {data.filters.toc.map((filter, index) => (
                  <MenuItem value={filter.id} key={index}>
                    {filter.label}
                  </MenuItem>
                ))}
              </Select>
            </FormControl>
            <FormControl sx={{ m: 1, minWidth: 120 }}>
              <InputLabel id="topic-label">Topics(s)</InputLabel>
              <Select
                value={filters.tags}
                className={styles.filterBtn}
                labelId="topic-label"
                onChange={handleTopic}
              >
                {data.filters.tags.map((label, index) => (
                  <MenuItem value={label} key={index}>
                    {label}
                  </MenuItem>
                ))}
              </Select>
            </FormControl>
            <FormControl sx={{ m: 1, minWidth: 120 }}>
              <InputLabel id="hardware-label">Hardware</InputLabel>
              <Select
                value={filters.hardware}
                className={styles.filterBtn}
                labelId="hardware-label"
                onChange={handleHardware}
              >
                {data.filters.hardware.map((label, index) => (
                  <MenuItem value={label} key={index}>
                    {label}
                  </MenuItem>
                ))}
              </Select>
            </FormControl>
            <FormControl sx={{ m: 1, minWidth: 120 }}>
              <InputLabel id="category-label">Category</InputLabel>
              <Select
                value={filters.category}
                className={styles.filterBtn}
                labelId="category-label"
                onChange={handleCategory}
              >
                {data.filters.category.map((label, index) => (
                  <MenuItem value={label} key={index}>
                    {label}
                  </MenuItem>
                ))}
              </Select>
            </FormControl>
          </Grid>
          <Grid item xs={3}>
            <ToggleButtonGroup
              className={styles.lvlBtn}
              color="success"
              value={filters.lvl}
              exclusive
              onChange={handleLevel}
              aria-label="text alignment"
            >
              <ToggleButton value="left" aria-label="left aligned">
                Beginner
              </ToggleButton>
              <ToggleButton value="center" aria-label="centered">
                Confirmed
              </ToggleButton>
              <ToggleButton value="right" aria-label="right aligned">
                Expert
              </ToggleButton>
            </ToggleButtonGroup>
          </Grid>
        </Grid>
      </Paper>
      {selection !== '' ? (
        <CardGrid selection={data.tuto} test={selection} />
      ) : (
        ''
      )}
    </div>
  );
}
