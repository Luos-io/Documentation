import React, { useState } from 'react';
import Lightbox from 'react-image-lightbox';
import { useColorMode } from '@docusaurus/theme-common';
import 'react-image-lightbox/style.css';

const Image = (props) => {
  const [isOpen, setIsOpen] = useState();
  const { isDarkTheme } = useColorMode();
  const source = isDarkTheme
    ? props.darkSrc === undefined
      ? props.src
      : props.darkSrc
    : props.src;
  const height = props.height === undefined ? '100%' : props.height;
  return (
    <div>
      <img
        className="imgPreview"
        src={source}
        onClick={() => setIsOpen(true)}
        height={height}
        alt={props.alt ? props.alt : 'luos_img'}
      />
      {isOpen && (
        <Lightbox
          mainSrc={props.darkSrc ? props.darkSrc : source}
          onCloseRequest={() => setIsOpen(false)}
        />
      )}
    </div>
  );
};

export default Image;
