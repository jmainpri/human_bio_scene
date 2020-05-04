GITHUB=https://github.com/jmainpri
BRANCH=bionic-beaver-melodic-dev
git clone $GITHUB/human_bio_octomap.git
git clone $GITHUB/human_bio_urdf.git
git clone $GITHUB/human_bio_scene.git
git clone $GITHUB/human_bio_dataset.git
cd human_bio_octomap && git checkout $BRANCH && cd ..
