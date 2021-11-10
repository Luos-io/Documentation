/**
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */
import React from 'react';
import Layout from '@theme/Layout';
import Translate, { translate } from '@docusaurus/Translate';
import { Redirect } from '@docusaurus/router';
import { useLocation } from 'react-router-dom';

function NotFound() {
  const location = useLocation();
  console.log(location);
  if (location.pathname.includes('/pages/')) {
    let newPathName = location.pathname.replace('/pages/', '/docs/');
    console.log(newPathName);
    return <Redirect to={newPathName} />;
  }

  return <Redirect to="/" />;
}

export default NotFound;
