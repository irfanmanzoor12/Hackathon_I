/**
 * MDX Components - Global component registration for Docusaurus
 * Components registered here are available in all MDX files
 */

import React from 'react';
import MDXComponents from '@theme-original/MDXComponents';
import ChapterActionsComponent from '@site/src/components/ChapterActions/ChapterActions';

function ChapterActions(props) {
  const [isMounted, setIsMounted] = React.useState(false);

  React.useEffect(() => {
    setIsMounted(true);
  }, []);

  if (!isMounted) {
    return <div style={{padding: '1rem', background: '#f5f5f5', borderRadius: '8px', margin: '1rem 0'}}>Loading...</div>;
  }

  return <ChapterActionsComponent {...props} />;
}

export default {
  ...MDXComponents,
  ChapterActions,
};
