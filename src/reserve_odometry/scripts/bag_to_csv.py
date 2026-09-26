#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path
from collections import defaultdict

import rosbag2_py

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def flatten_ros_message(msg, prefix=""):
    """
    Преобразует ROS 2 message в плоский словарь.

    Например:
        header.stamp.sec       -> 123
        header.stamp.nanosec   -> 456
        header.frame_id        -> base_link
        velocity               -> 10.5

    Вложенные сообщения разворачиваются рекурсивно.
    Массивы разворачиваются как:
        position_covariance[0]
        position_covariance[1]
        ...
    """

    result = {}

    if hasattr(msg, "get_fields_and_field_types"):
        fields = msg.get_fields_and_field_types()

        for field_name in fields:
            value = getattr(msg, field_name)

            key = f"{prefix}.{field_name}" if prefix else field_name

            if hasattr(value, "get_fields_and_field_types"):
                result.update(flatten_ros_message(value, key))

            elif isinstance(value, (list, tuple)):
                for i, item in enumerate(value):
                    item_key = f"{key}[{i}]"

                    if hasattr(item, "get_fields_and_field_types"):
                        result.update(
                            flatten_ros_message(item, item_key)
                        )
                    else:
                        result[item_key] = item

            elif hasattr(value, "__iter__") and not isinstance(
                value, (str, bytes)
            ):
                # numpy-подобные массивы / ROS sequences
                try:
                    values = list(value)

                    for i, item in enumerate(values):
                        result[f"{key}[{i}]"] = item

                except TypeError:
                    result[key] = value

            else:
                result[key] = value

    else:
        result[prefix] = msg

    return result


def topic_to_filename(topic_name: str) -> str:
    """
    /vehicle/front_bogie_velocity
        ->
    vehicle_front_bogie_velocity.csv
    """

    name = topic_name.strip("/").replace("/", "_")

    if not name:
        name = "root"

    return f"{name}.csv"


def find_bag_directories(input_root: Path):
    """
    Ищет ROS 2 bag директории рекурсивно.

    Ожидается структура типа:

    data/
      30618_xxxxxxxx/
        metadata.yaml
        30618_xxxxxxxx_0.db3

      30618_yyyyyyyy/
        metadata.yaml
        30618_yyyyyyyy_0.db3
    """

    bag_dirs = []

    for metadata_file in input_root.rglob("metadata.yaml"):
        bag_dirs.append(metadata_file.parent)

    return sorted(set(bag_dirs))


def read_bag(bag_dir: Path):
    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_dir),
        storage_id="sqlite3"
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )

    reader.open(
        storage_options,
        converter_options
    )

    topics_info = reader.get_all_topics_and_types()

    topic_types = {
        topic.name: topic.type
        for topic in topics_info
    }

    return reader, topic_types


def export_bag(
    bag_dir: Path,
    input_root: Path,
    output_root: Path
):
    print()
    print("=" * 80)
    print(f"Bag: {bag_dir}")

    relative_dir = bag_dir.relative_to(input_root)

    bag_output_dir = output_root / relative_dir
    bag_output_dir.mkdir(parents=True, exist_ok=True)

    reader, topic_types = read_bag(bag_dir)

    print("Топики:")

    for topic, msg_type in topic_types.items():
        print(f"  {topic}")
        print(f"      {msg_type}")

    # Сначала собираем сообщения по топикам.
    #
    # Для датасета такого размера это всё ещё разумно,
    # но CSV при этом пишутся отдельно для каждого топика.
    rows_by_topic = defaultdict(list)

    message_classes = {}

    for topic_name, type_name in topic_types.items():
        try:
            message_classes[topic_name] = get_message(type_name)

        except Exception as exc:
            print(
                f"[WARNING] Не удалось загрузить тип "
                f"{type_name} для {topic_name}: {exc}"
            )

    total_messages = 0

    while reader.has_next():
        topic_name, raw_data, timestamp = reader.read_next()

        if topic_name not in message_classes:
            continue

        msg_class = message_classes[topic_name]

        try:
            msg = deserialize_message(
                raw_data,
                msg_class
            )

        except Exception as exc:
            print(
                f"[WARNING] Ошибка десериализации "
                f"{topic_name}: {exc}"
            )
            continue

        row = {
            "bag_timestamp_ns": timestamp,
            "bag_timestamp_sec": timestamp / 1e9,
        }

        message_data = flatten_ros_message(msg)

        row.update(message_data)

        # Дополнительно создадим удобное поле header_stamp_ns
        # для сообщений, у которых есть std_msgs/Header.

        sec = message_data.get("header.stamp.sec")
        nanosec = message_data.get("header.stamp.nanosec")

        if sec is not None and nanosec is not None:
            row["header_stamp_ns"] = (
                int(sec) * 1_000_000_000 +
                int(nanosec)
            )

            row["header_stamp_sec"] = (
                int(sec) +
                int(nanosec) / 1e9
            )

        rows_by_topic[topic_name].append(row)

        total_messages += 1

    print(f"Прочитано сообщений: {total_messages}")

    # Сохраняем каждый топик в отдельный CSV.

    for topic_name, rows in rows_by_topic.items():

        if not rows:
            continue

        filename = topic_to_filename(topic_name)
        output_file = bag_output_dir / filename

        # Собираем полный набор колонок.
        # Это важно для сообщений с массивами/вложенными полями.

        fieldnames = []

        seen = set()

        preferred_fields = [
            "bag_timestamp_ns",
            "bag_timestamp_sec",
            "header_stamp_ns",
            "header_stamp_sec",
        ]

        for field in preferred_fields:
            if any(field in row for row in rows):
                fieldnames.append(field)
                seen.add(field)

        for row in rows:
            for field in row.keys():
                if field not in seen:
                    seen.add(field)
                    fieldnames.append(field)

        with output_file.open(
            "w",
            newline="",
            encoding="utf-8"
        ) as file:

            writer = csv.DictWriter(
                file,
                fieldnames=fieldnames
            )

            writer.writeheader()
            writer.writerows(rows)

        print(
            f"  CSV: {output_file} "
            f"({len(rows)} сообщений)"
        )


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Рекурсивный экспорт ROS 2 bag "
            "в отдельные CSV для каждого топика."
        )
    )

    parser.add_argument(
        "input",
        type=Path,
        help="Корневая директория с ROS 2 bag"
    )

    parser.add_argument(
        "output",
        type=Path,
        help="Директория для CSV"
    )

    args = parser.parse_args()

    input_root = args.input.resolve()
    output_root = args.output.resolve()

    if not input_root.exists():
        raise FileNotFoundError(
            f"Директория не существует: {input_root}"
        )

    bag_dirs = find_bag_directories(input_root)

    if not bag_dirs:
        print(
            f"В {input_root} не найдено ни одного "
            f"metadata.yaml"
        )
        return

    print(f"Найдено bag: {len(bag_dirs)}")
    print(f"Вход:  {input_root}")
    print(f"Выход: {output_root}")

    success = 0
    failed = 0

    for bag_dir in bag_dirs:
        try:
            export_bag(
                bag_dir,
                input_root,
                output_root
            )

            success += 1

        except Exception as exc:
            failed += 1

            print()
            print(
                f"[ERROR] Не удалось обработать "
                f"{bag_dir}"
            )
            print(exc)

    print()
    print("=" * 80)
    print("Готово.")
    print(f"Успешно: {success}")
    print(f"Ошибок:   {failed}")


if __name__ == "__main__":
    main()