class ActorCleanup:

    @staticmethod
    def cleanup_existing_actors(world):

        actors = list(
            world.get_actors().filter('controller.ai.walker')
        ) + list(
            world.get_actors().filter('sensor.*')
        ) + list(
            world.get_actors().filter('walker.pedestrian.*')
        ) + list(
            world.get_actors().filter('vehicle.*')
        )

        destroyed = ActorCleanup.destroy_actors(actors)

        if destroyed > 0:
            print(f"Startup cleanup destroyed {destroyed} leftover actors")

    @staticmethod
    def destroy_actors(actors):

        destroyed = 0

        for actor in actors:

            destroyed += ActorCleanup.destroy_actor(actor)

        return destroyed

    @staticmethod
    def destroy_actor(actor):

        if actor is None:
            return 0

        try:
            if not actor.is_alive:
                return 0

            actor_id = actor.id
            actor_type = actor.type_id

            if actor_type == 'controller.ai.walker':
                actor.stop()

            actor.destroy()

            print(f"Destroyed actor {actor_id}: {actor_type}")

            return 1

        except Exception as exc:
            print(f"Actor cleanup skipped: {exc}")

        return 0
