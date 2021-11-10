/**
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */
import React from 'react';
import { Redirect } from '@docusaurus/router';
import { useLocation } from 'react-router-dom';

function NotFound() {
  const location = useLocation();
  if (location.pathname.includes('/pages/')) {
    let newPathName = location.pathname.replace('/pages/', '/docs/');
    return <Redirect to={newPathName} />;
  }

  return <Redirect to="/" />;
}

export default NotFound;
