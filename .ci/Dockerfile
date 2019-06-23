FROM acutronicrobotics/mara:dashing-ci

COPY ./travis.sh /travis.sh
COPY ./ /tmp/mara/

RUN /travis.sh

CMD ["bash"]
