find . -type f \( -name "*.cpp" -o -name "*.hpp" \) | xargs clang-format -i -style=Chromium
find . -maxdepth 1 -name "*py"|xargs python3 -m autoflake -i --remove-all-unused-imports --remove-unused-variables --ignore-init-module-imports
python3 -m black .  --line-length 100
python3 -m isort .
