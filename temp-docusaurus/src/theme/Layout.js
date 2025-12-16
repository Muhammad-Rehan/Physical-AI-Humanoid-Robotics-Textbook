import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import RagChatbot from '../components/RagChatbot';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props} />
      <RagChatbot />
    </>
  );
}