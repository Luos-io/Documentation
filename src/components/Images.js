import React, { Component } from 'react';
import Lightbox from 'react-image-lightbox';
import useThemeContext from '@theme/hooks/useThemeContext';
import 'react-image-lightbox/style.css';

export default class Image extends Component {
  constructor(props) {
    super(props);

    this.state = {
      isOpen: false,
      src: props.src,
      darkSrc: props.darkSrc,
    };
  }

  render() {
    const { isOpen, src, darkSrc } = this.state;
    const { isDarkTheme } = useThemeContext();
    // const source = isDarkTheme ? (darkSrc === '' ? src : srcdarkSrc) : src;
    return (
      <div>
        <img src={src} onClick={() => this.setState({ isOpen: true })} />
        {isOpen && (
          <Lightbox
            mainSrc={src}
            onCloseRequest={() => this.setState({ isOpen: false })}
          />
        )}
      </div>
    );
  }
}
