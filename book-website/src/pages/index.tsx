import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';
import PersonalizedContent from '../components/PersonalizedContent'; // Import PersonalizedContent

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">By Ashbah Sami - {siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/introduction">
            Start Reading
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <div className="container margin-vert--xl">
          <div className="row">
            <div className="col col--8 col--offset-2">
              <PersonalizedContent /> {/* Integrated PersonalizedContent */}
              <Heading as="h2" className="margin-top--lg">
                Welcome to your AI-Powered Robotics Textbook!
              </Heading>
              <p>
                This textbook is designed to adapt to your learning style and technical background.
                Sign up and share your software and hardware experience to get a truly personalized learning experience.
              </p>
              <p>
                Explore topics from ROS 2 to advanced Vision-Language-Action (VLA) models, with content tailored just for you.
              </p>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
