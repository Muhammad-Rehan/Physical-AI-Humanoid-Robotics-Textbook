import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  image: string;
  description: ReactNode;
  weeks: string;
  link: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Module 1: ROS2',
    image: '/img/ROS2.jpg',
    description: (
      <>
        Explore the fundamentals of ROS2, from basic concepts to advanced robotic navigation.
      </>
    ),
    weeks: 'Week 1 to 4',
    link: '/docs/module1-ros2/introduction',
  },
  {
    title: 'Module 2: Gazebo',
    image: '/img/gazebo.jpg',
    description: (
      <>
        Dive into Gazebo for realistic robot simulations and environment building.
      </>
    ),
    weeks: 'Week 5 to 7',
    link: '/docs/module2-gazebo/physics-simulation',
  },
  {
    title: 'Module 3: Isaac Sim',
    image: '/img/ISAAC_SIM.png',
    description: (
      <>
        Learn to leverage NVIDIA Isaac Sim for high-fidelity simulation and synthetic data generation.
      </>
    ),
    weeks: 'Week 8 to 12',
    link: '/docs/module3-isaac/isaac-sim',
  },
  {
    title: 'Module 4: VLA',
    image: '/img/VLA.png',
    description: (
      <>
        Understand Visual-Language-Action models and their application in robotics.
      </>
    ),
    weeks: 'Week 13 to 14',
    link: '/docs/module4-vla/cognitive-planning-llm',
  },
];

function Feature({title, image, description, weeks, link}: FeatureItem) {
  return (
    <Link to={link} className={clsx(styles.featureCardLink)}>
      <div className={clsx(styles.featureCard)}>
        <div className="text--center">
          <img src={image} className={styles.featureImage} alt={title} />
        </div>
        <div className="text--center padding-horiz--md">
          <Heading as="h3">{title}</Heading>
          <p>{description}</p>
        </div>
        <div className={styles.weeksTagline}>{weeks}</div>
      </div>
    </Link>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.featuresGrid}>
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
