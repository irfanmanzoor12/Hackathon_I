/**
 * MDX Components - Global component registration for Docusaurus
 * Components registered here are available in all MDX files
 */

import React from 'react';
import MDXComponents from '@theme-original/MDXComponents';
import ChapterActions from '@site/src/components/ChapterActions';

export default {
  ...MDXComponents,
  ChapterActions,
};
