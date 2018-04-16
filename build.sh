set -e

export PATH=$PATH:/home/travis/.cargo/bin;

echo 'Building book...'
mdbook build

# Only run this on the master branch
if [ "$TRAVIS_PULL_REQUEST" != "false" ] || 
   [ "$TRAVIS_BRANCH" != "master" ]; then
   exit 0
fi

echo -e "${CYAN}Initializing Git${NC}"
git init
git config user.name "Mathieu David"
git config user.email "mathieudavid@mathieudavid.org"

git remote add upstream "https://$GH_TOKEN@github.com/Ecam-Eurobot/Tutorials.git"
git fetch upstream --quiet
git reset upstream/gh-pages --quiet

touch .

echo -e "${CYAN}Pushing changes to gh-pages${NC}"
git add -A . 
git commit -m "rebuild pages at ${rev}" --quiet
git push -q upstream HEAD:gh-pages --quiet
echo -e "${GREEN}Deployed docs to GitHub Pages${NC}"