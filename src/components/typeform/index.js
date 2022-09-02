import { Widget } from '@typeform/embed-react';
import React from 'react';
import '@typeform/embed/build/css/widget.css';

const Form = (props) => (
  <div>
    <Widget id={props.id} style={{ height: '500px' }} lazy />
  </div>
);

export default Form;
