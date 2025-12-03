// frontend/src/pages/index.js

import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Chatbot from '../components/Chatbot'; // T4.4: Import the Chatbot Component
import styles from './index.module.css'; // T4.4: Import custom styling for the page

// Note: HomepageHeader component ko hata diya gaya hai.

function HomepageContent() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <section className={styles.contentSection}>
      <div className={styles.chatWrapper}>
        <h1 className={styles.mainTitle}>{siteConfig.title}</h1>
        <p className={styles.tagline}>Start asking questions about your textbook content.</p>
        <Chatbot /> {/* T4.4: Render the Chatbot Component */}
      </div>
    </section>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Home | ${siteConfig.title}`}
      description="RAG Chatbot Interface for Textbook Knowledge">
      <main>
        <HomepageContent />
      </main>
    </Layout>
  );
}