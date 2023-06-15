FROM node:18-alpine as development

# set working directory
WORKDIR /app

RUN apk update && \
    apk upgrade

# copy needed files
COPY package*.json ./
COPY . .

RUN npm i --legacy-peer-deps

# RUN npm i --legacy-peer-deps

CMD npm start


FROM node:18-alpine as pre-production

# set working directory
WORKDIR /app

RUN apk update && \
    apk upgrade

COPY . .

RUN npm i

RUN npm run build


# pull official base image
FROM node:18-alpine as production

# set working directory
WORKDIR /app

RUN apk update && \
    apk upgrade

RUN npm install -g serve

COPY --from=pre-production /app/build /app/build

ENTRYPOINT serve -s build -l 4000 