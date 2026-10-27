# Перенос программной части в общий репозиторий

Архив программной части содержит код для корня репозитория. После распаковки должны быть видны `ros2_ws`, `docker`, `scripts`, `scenarios`, `.github`, `README.md` и остальные файлы. Сам ZIP в репозиторий с кодом загружать не требуется.

## Если общий репозиторий уже создан

Получите его актуальную копию. Вместо `URL_ОБЩЕГО_РЕПОЗИТОРИЯ` нужен реальный адрес:

```sh
git clone 'URL_ОБЩЕГО_РЕПОЗИТОРИЯ' project2600
cd project2600
git switch -c feature/stonefish-ardusub-integration
```

Перенесите файлы из распакованного архива в эту папку с сохранением структуры. Существующие README, правила Git и workflow нужно объединить с изменениями команды. Текущий CI ожидает папки `ros2_ws`, `docker` и `scripts` в корне.

Повторите локальные проверки и запуск ROS2 из README. Затем выберите файлы этого изменения:

```sh
git status --short
git add .gitignore .dockerignore pyproject.toml README.md CONTRIBUTING.md
git add .github/workflows/ci.yml ros2_ws/src/auv_diagnostics docker scripts scenarios
git add docs
git add reports SOURCE_SHA256SUMS.txt
git diff --cached --stat
git diff --cached --check
git commit -m "Connect Stonefish and ArduSub to ROS2 monitoring"
git push -u origin feature/stonefish-ardusub-integration
```

Перед коммитом список должен содержать только файлы этой работы. После отправки ветки откройте Pull Request и дождитесь результата GitHub Actions. Ссылку на PR или коммит можно добавить в карточку Wekan.

## Проверка содержимого архива

Из корня распакованной папки:

```sh
shasum -a 256 -c SOURCE_SHA256SUMS.txt
```

Для каждого файла ожидается `OK`. Список относится к переданной версии файлов. При последующих изменениях кода его нужно обновить или удалить из нового изменения.

Материалы презентации, персональные фотографии, примеры менеджеров и карточки отчётности в архив исходного кода не включены.
