import React from 'react';

export default function Tooltip(props) {
  return (
    <a>
      <span className="cust_tooltip">
        {props.children}
        <span className="cust_tooltiptext">{props.def}</span>
      </span>
    </a>
  );
}
